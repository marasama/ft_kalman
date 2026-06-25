#![feature(generic_const_exprs)]
#![allow(incomplete_features)]
use crate::filters::{KalmanModel, UKF_ObservationModel, UKF_TransitionModel, UKF};
use matrix::matrix::{funcs::inverse::identity_matrix, Matrix};
use matrix::vector::Vector;
use num_traits::Float;
use std::ops::{AddAssign, Mul, SubAssign};

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    UKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
    [(); N_Z * N_X]:,
    [(); N_Z * (2 * N_X + 1)]:,
    [(); N_X * (2 * N_X + 1)]:,
    [(); (2 * N_X + 1) * (2 * N_X + 1)]:,
{
    fn calculate_sigma_points(
        &self,
        x: &Vector<K, N_X>,
        P: &Matrix<K, N_X, N_X>,
    ) -> Matrix<K, N_X, { 2 * N_X + 1 }>
    where
        [(); N_X * { 2 * N_X + 1 }]:,
    {
        let mut data = [K::zero(); N_X * (2 * N_X + 1)];
        let scale: K = K::from(N_X).unwrap() + self.lambda;
        let num_cols = N_X * 2 + 1;
        let sqrt_mat = P.scl_mat_ref(scale).cholesky();
        for r in 0..N_X {
            // First columns is state vector itself
            data[r * num_cols] = x.data[r];
            for i in 0..N_X {
                let delta = sqrt_mat.data[r * N_X + i];

                data[r * num_cols + i + 1] = x.data[r] + delta;
                data[r * num_cols + i + N_X + 1] = x.data[r] - delta;
            }
        }
        Matrix { data }
    }

    fn calculate_weights(
        w_0: K,
        w_0_c: K,
        w_i: K,
    ) -> (
        Vector<K, { 2 * N_X + 1 }>,
        Matrix<K, { 2 * N_X + 1 }, { 2 * N_X + 1 }>,
    ) {
        let mut diagonal = identity_matrix::<K, { 2 * N_X + 1 }>();
        diagonal.scl(w_i);
        diagonal.data[0] = w_0_c;
        let mut new_data = [w_i; 2 * N_X + 1];
        new_data[0] = w_0;
        (Vector { data: new_data }, diagonal)
    }

    pub fn new(
        F: UKF_TransitionModel<K, N_X, N_U>,
        H: UKF_ObservationModel<K, N_X, N_Z>,
        G: Option<Matrix<K, N_X, N_U>>,
        alpha: K,
        beta: K,
        kappa: K,
    ) -> Self {
        let n = K::from(N_X).unwrap();

        if matches!(F, UKF_TransitionModel::NonLinear { .. }) && G.is_some() {
            panic!("G is set but Nonlinear Transition Model does not use it!");
        }

        let lambda = num_traits::pow(alpha, 2) * (n + kappa) - n;
        let (weights, weights_diag) = UKF::<K, N_X, N_Z, N_U>::calculate_weights(
            lambda / (n + lambda),
            lambda / (n + lambda) + (K::one() - num_traits::pow(alpha, 2) + beta),
            K::one() / (K::from(2).unwrap() * (n + lambda)),
        );

        UKF {
            F,
            H,
            G,
            sigma_points_prior: Matrix::<K, N_X, { 2 * N_X + 1 }>::zeros(),
            lambda,
            weights,
            weights_diag,
        }
    }
}

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    KalmanModel<K, N_X, N_Z, N_U> for UKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_X]:,
    [(); N_Z * N_Z]:,
    [(); N_Z * (2 * N_X + 1)]:,
    [(); (2 * N_X + 1) * N_Z]:,
    [(); N_X * (2 * N_X + 1)]:,
    [(); (2 * N_X + 1) * N_X]:,
    [(); (2 * N_X + 1) * (2 * N_X + 1)]:,
{
    fn state_dim(&self) -> usize {
        N_X
    }
    fn meas_dim(&self) -> usize {
        N_Z
    }
    fn update(
        &self,
        x_prior: &Vector<K, N_X>,
        z: &Vector<K, N_Z>,
        P_prior: &Matrix<K, N_X, N_X>,
        R: &Matrix<K, N_Z, N_Z>,
    ) -> (Vector<K, N_X>, Matrix<K, N_X, N_X>, Matrix<K, N_X, N_Z>) // x, P, K
    {
        let meas_space = match &self.H {
            UKF_ObservationModel::Linear { H } => H.mul_mat(&self.sigma_points_prior),
            UKF_ObservationModel::NonLinear { h } => h(&self.sigma_points_prior),
        };

        let pred_meas_mean = meas_space.mul_vec(&self.weights);
        let tmp_subt = meas_space.sub_each_col(&pred_meas_mean);
        let meas_space_cov = tmp_subt
            .mul_mat(&self.weights_diag)
            .mul_mat(&tmp_subt.transpose())
            .add_mat_ref(R);
        let cross_cov_state_meas = self
            .sigma_points_prior
            .sub_each_col(x_prior)
            .mul_mat(&self.weights_diag)
            .mul_mat(&tmp_subt.transpose());
        let K_n = cross_cov_state_meas.mul_mat(&meas_space_cov.inverse().unwrap());
        let x = x_prior.add_vec_ref(&K_n.mul_vec(&z.sub_vec_ref(&pred_meas_mean)));
        let P = P_prior.sub_mat_ref(&K_n.mul_mat(&meas_space_cov).mul_mat(&K_n.transpose()));
        (x, P, K_n)
    }
    fn predict(
        &mut self,
        x: &Vector<K, N_X>,
        u: &Option<Vector<K, N_U>>,
        P: &Matrix<K, N_X, N_X>,
        Q: &Matrix<K, N_X, N_X>,
    ) -> (Vector<K, N_X>, Matrix<K, N_X, N_X>) // x_prior, P_prior
    {
        let sigma_points = self.calculate_sigma_points(x, P);
        self.sigma_points_prior = match &self.F {
            UKF_TransitionModel::Linear { F, .. } => {
                let mut transformed = F.mul_mat(&sigma_points);
                if let (Some(g), Some(v)) = (&self.G, u) {
                    transformed = transformed.add_each_col(&g.mul_vec(v))
                } else if self.G.is_some() || u.is_some() {
                    panic!("Control Vector or Input Variable initialized wrong!");
                }
                transformed
            }
            UKF_TransitionModel::NonLinear { f } => f(&sigma_points, u),
        };
        let x_prior = self.sigma_points_prior.mul_vec(&self.weights);
        let tmp_subt = self.sigma_points_prior.sub_each_col(&x_prior);
        let P_prior = tmp_subt
            .mul_mat(&self.weights_diag)
            .mul_mat(&tmp_subt.transpose())
            .add_mat_ref(Q);
        (x_prior, P_prior)
    }
}

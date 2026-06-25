use crate::filters::{EKF_ObservationModel, EKF_TransitionModel, KalmanModel, EKF};
use matrix::matrix::{funcs::inverse::identity_matrix, Matrix};
use matrix::vector::Vector;
use num_traits::Float;
use std::ops::{AddAssign, SubAssign};

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    EKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
    [(); N_X * N_U]:,
{
    fn update_kalman_gain(
        &self,
        x_prior: &Vector<K, N_X>,
        P_prior: &Matrix<K, N_X, N_X>,
        R: &Matrix<K, N_Z, N_Z>,
    ) -> Matrix<K, N_X, N_Z> {
        match &self.H {
            EKF_ObservationModel::Linear { H, H_transpose } => {
                let tmp = H.mul_mat(P_prior).mul_mat(H_transpose).add_mat_ref(R);
                P_prior
                    .mul_mat(H_transpose)
                    .mul_mat(&tmp.inverse().unwrap())
            }
            EKF_ObservationModel::NonLinear { jac, .. } => {
                let jacobian = jac(x_prior);
                let tmp = jacobian
                    .mul_mat(P_prior)
                    .mul_mat(&jacobian.transpose())
                    .add_mat_ref(R);
                P_prior
                    .mul_mat(&jacobian.transpose())
                    .mul_mat(&tmp.inverse().unwrap())
            }
        }
    }

    fn update_estimation_covariance(
        &self,
        x_prior: &Vector<K, N_X>,
        K_n: &Matrix<K, N_X, N_Z>,
        R: &Matrix<K, N_Z, N_Z>,
        P_prior: &Matrix<K, N_X, N_X>,
    ) -> Matrix<K, N_X, N_X> {
        let joseph = K_n.mul_mat(R).mul_mat(&K_n.transpose());
        let main_part = match &self.H {
            EKF_ObservationModel::Linear { H, .. } => self.I.sub_mat_ref(&K_n.mul_mat(H)),
            EKF_ObservationModel::NonLinear { jac, .. } => {
                self.I.sub_mat_ref(&K_n.mul_mat(&jac(x_prior)))
            }
        };
        main_part
            .mul_mat(P_prior)
            .mul_mat(&main_part.transpose())
            .add_mat_ref(&joseph)
    }

    pub fn new(
        F: EKF_TransitionModel<K, N_X, N_Z, N_U>,
        H: EKF_ObservationModel<K, N_X, N_Z>,
    ) -> Self {
        EKF {
            F,
            H,
            I: identity_matrix::<K, N_X>(),
        }
    }
}

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    KalmanModel<K, N_X, N_Z, N_U> for EKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
    [(); N_X * N_U]:,
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
        let K_n = self.update_kalman_gain(x_prior, P_prior, R);
        let x = match &self.H {
            EKF_ObservationModel::Linear { H, .. } => K_n
                .mul_vec(&(z.sub_vec_ref(&H.mul_vec(x_prior))))
                .add_vec_ref(x_prior),
            EKF_ObservationModel::NonLinear { f, .. } => K_n
                .mul_vec(&(z.sub_vec_ref(&f(x_prior))))
                .add_vec_ref(x_prior),
        };
        let P = self.update_estimation_covariance(x_prior, &K_n, R, P_prior);

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
        match &self.F {
            EKF_TransitionModel::Linear { F, F_transpose, G } => {
                let x_prior = F.mul_vec(x);
                let P_prior = F.mul_mat(P).mul_mat(F_transpose).add_mat_ref(Q);
                match (G, u) {
                    (Some(g), Some(v)) => (x_prior.add_vec_ref(&g.mul_vec(v)), P_prior),
                    (Some(_), None) => {
                        panic!("Control Matrix is set but Input vector is empty!")
                    }
                    (None, Some(_)) => {
                        panic!("Control Vector provided but G is not set!");
                    }
                    (None, None) => (x_prior, P_prior),
                }
            }
            EKF_TransitionModel::NonLinear { f, jac } => {
                let jacobian = jac(x, u);
                (
                    f(x, u),
                    jacobian
                        .mul_mat(P)
                        .mul_mat(&jacobian.transpose())
                        .add_mat_ref(Q),
                )
            }
        }
    }
}

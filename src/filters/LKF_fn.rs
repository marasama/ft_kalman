use crate::filters::{KalmanModel, LKF};
use matrix::matrix::{funcs::inverse::identity_matrix, Matrix};
use matrix::vector::Vector;
use num_traits::Float;
use std::ops::{AddAssign, SubAssign};

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    LKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
    [(); N_X * N_U]:,
{
    fn update_kalman_gain(
        &self,
        P_prior: &Matrix<K, N_X, N_X>,
        R: &Matrix<K, N_Z, N_Z>,
    ) -> Matrix<K, N_X, N_Z> {
        let mut gain = self
            .H
            .mul_mat(P_prior)
            .mul_mat(&self.H_transpose)
            .add_mat_ref(R);
        P_prior
            .mul_mat(&self.H_transpose)
            .mul_mat(&gain.inverse().unwrap())
    }

    fn update_estimation_covariance(
        &self,
        K_n: &Matrix<K, N_X, N_Z>,
        R: &Matrix<K, N_Z, N_Z>,
        P_prior: &Matrix<K, N_X, N_X>,
    ) -> Matrix<K, N_X, N_X> {
        let P_joseph = K_n.mul_mat(R).mul_mat(&K_n.transpose());
        let main_part = self.I.sub_mat_ref(&K_n.mul_mat(&self.H));
        main_part
            .mul_mat(P_prior)
            .mul_mat(&main_part.transpose())
            .add_mat_ref(&P_joseph)
    }

    pub fn new(
        F: Matrix<K, N_X, N_X>,
        H: Matrix<K, N_Z, N_X>,
        G: Option<Matrix<K, N_X, N_U>>,
    ) -> Self {
        LKF {
            F_transpose: F.transpose(),
            F,
            H_transpose: H.transpose(),
            H,
            G,
            I: identity_matrix::<K, N_X>(),
        }
    }
}

impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    KalmanModel<K, N_X, N_Z, N_U> for LKF<K, N_X, N_Z, N_U>
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
        let K_n = self.update_kalman_gain(P_prior, R);
        let x = K_n
            .mul_vec(&(z.sub_vec_ref(&self.H.mul_vec(x_prior))))
            .add_vec_ref(x_prior);
        let P = self.update_estimation_covariance(&K_n, R, P_prior);
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
        let P_prior = self.F.mul_mat(P).mul_mat(&self.F_transpose).add_mat_ref(Q);
        let x_prior = self.F.mul_vec(x);
        match (&self.G, u) {
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
}
impl<K: Float + AddAssign + SubAssign, const N_X: usize, const N_Z: usize, const N_U: usize>
    LKF<K, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
    [(); N_X * N_U]:,
{
    pub fn update_with<const N_Z_NEW: usize>(
        &self,
        x_prior: &Vector<K, N_X>,
        z: &Vector<K, N_Z_NEW>,
        u: &Option<Vector<K, N_U>>,
        P_prior: &Matrix<K, N_X, N_X>,
        R: &Matrix<K, N_Z_NEW, N_Z_NEW>,
        H: &Matrix<K, N_Z_NEW, N_X>,
    ) -> (Vector<K, N_X>, Matrix<K, N_X, N_X>, Matrix<K, N_X, N_Z_NEW>)
    where
        [(); N_Z_NEW * N_Z_NEW]:,
        [(); N_Z_NEW * N_X]:,
        [(); N_X * N_Z_NEW]:,
    {
        let gain = H.mul_mat(P_prior).mul_mat(&H.transpose()).add_mat_ref(R);
        let K_n = P_prior
            .mul_mat(&H.transpose())
            .mul_mat(&gain.inverse().unwrap());
        let x = K_n
            .mul_vec(&(z.sub_vec_ref(&H.mul_vec(x_prior))))
            .add_vec_ref(x_prior);
        let P_joseph = K_n.mul_mat(R).mul_mat(&K_n.transpose());
        let main_part = self.I.sub_mat_ref(&K_n.mul_mat(H));
        let P = main_part
            .mul_mat(P_prior)
            .mul_mat(&main_part.transpose())
            .add_mat_ref(&P_joseph);
        (x, P, K_n)
    }
}

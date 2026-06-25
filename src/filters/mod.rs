#![allow(non_snake_case)]
#![allow(unused)]
pub mod EKF_fn;
pub mod LKF_fn;
pub mod UKF_fn;

use matrix::matrix::Matrix;
use matrix::vector::Vector;
use num_traits::Float;

pub trait KalmanModel<K: Float, const N_X: usize, const N_Z: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
{
    fn state_dim(&self) -> usize;
    fn meas_dim(&self) -> usize;
    fn predict(
        &mut self,
        x: &Vector<K, N_X>,
        u: &Option<Vector<K, N_U>>,
        P: &Matrix<K, N_X, N_X>,
        Q: &Matrix<K, N_X, N_X>,
    ) -> (Vector<K, N_X>, Matrix<K, N_X, N_X>); // x_prior, P_prior
    fn update(
        &self,
        x_prior: &Vector<K, N_X>,
        z: &Vector<K, N_Z>,
        P_prior: &Matrix<K, N_X, N_X>,
        R: &Matrix<K, N_Z, N_Z>,
    ) -> (Vector<K, N_X>, Matrix<K, N_X, N_X>, Matrix<K, N_X, N_Z>); // x, P, K
}

pub struct KalmanCore<
    K: Float,
    M: KalmanModel<K, N_X, N_Z, N_U>,
    const N_X: usize,
    const N_Z: usize,
    const N_U: usize,
> where
    [(); N_X * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
{
    pub model: M,
    pub x: Vector<K, N_X>,
    pub x_prior: Vector<K, N_X>,
    pub P: Matrix<K, N_X, N_X>,
    pub P_prior: Matrix<K, N_X, N_X>,
    pub K: Matrix<K, N_X, N_Z>,
    pub Q: Matrix<K, N_X, N_X>,
    pub R: Matrix<K, N_Z, N_Z>,
}

impl<
        K: Float,
        M: KalmanModel<K, N_X, N_Z, N_U>,
        const N_X: usize,
        const N_Z: usize,
        const N_U: usize,
    > KalmanCore<K, M, N_X, N_Z, N_U>
where
    [(); N_X * N_X]:,
    [(); N_X * N_Z]:,
    [(); N_Z * N_Z]:,
{
    pub fn state(&self) -> &Vector<K, N_X> {
        &self.x
    }

    pub fn covariance(&self) -> &Matrix<K, N_X, N_X> {
        &self.P
    }

    pub fn kalman_gain(&self) -> &Matrix<K, N_X, N_Z> {
        &self.K
    }

    pub fn step(&mut self, z: &Vector<K, N_Z>, u: &Option<Vector<K, N_U>>) {
        (self.x_prior, self.P_prior) = self.model.predict(&self.x, u, &self.P, &self.Q);
        (self.x, self.P, self.K) = self.model.update(&self.x_prior, z, &self.P_prior, &self.R);
    }

    pub fn new(
        model: M,
        x: Vector<K, N_X>,
        P: Matrix<K, N_X, N_X>,
        Q: Matrix<K, N_X, N_X>,
        R: Matrix<K, N_Z, N_Z>,
    ) -> Self {
        Self {
            model,
            x,
            x_prior: Vector::<K, N_X>::zeros(),
            P,
            P_prior: Matrix::<K, N_X, N_X>::zeros(),
            K: Matrix::<K, N_X, N_Z>::zeros(),
            Q,
            R,
        }
    }
}

// Unscented Kalman Filter -----------------------------------------------
pub enum UKF_TransitionModel<K: Float, const N_X: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * (2 * N_X + 1)]:,
{
    Linear {
        F: Matrix<K, N_X, N_X>,
    },
    NonLinear {
        f: fn(
            &Matrix<K, N_X, { 2 * N_X + 1 }>,
            &Option<Vector<K, N_U>>,
        ) -> Matrix<K, N_X, { 2 * N_X + 1 }>,
    },
}

pub enum UKF_ObservationModel<K: Float, const N_X: usize, const N_Z: usize>
where
    [(); N_Z * N_X]:,
    [(); N_Z * (2 * N_X + 1)]:,
    [(); N_X * (2 * N_X + 1)]:,
{
    Linear {
        H: Matrix<K, N_Z, N_X>,
    },

    NonLinear {
        h: fn(&Matrix<K, N_X, { 2 * N_X + 1 }>) -> Matrix<K, N_Z, { 2 * N_X + 1 }>,
    },
}

/// Unscented Kalman Filter
pub struct UKF<K: Float, const N_X: usize, const N_Z: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
    [(); N_Z * N_X]:,
    [(); N_Z * (2 * N_X + 1)]:,
    [(); N_X * (2 * N_X + 1)]:,
    [(); (2 * N_X + 1) * (2 * N_X + 1)]:,
{
    ///// State Vector
    //pub n_x: usize,
    ///// Measurement Vector
    //pub n_z: usize,
    ///// Input Vector
    //pub n_u: usize,
    /// Transition Model
    F: UKF_TransitionModel<K, N_X, N_U>,
    /// Observation Model
    H: UKF_ObservationModel<K, N_X, N_Z>,
    /// Control Matrix
    G: Option<Matrix<K, N_X, N_U>>,
    /// Sigma Points of the next step
    sigma_points_prior: Matrix<K, N_X, { 2 * N_X + 1 }>,
    /// Lambda calculated with tune parameters for calculating the sigma points
    lambda: K,
    /// Weights for calculating the sigma points
    weights: Vector<K, { 2 * N_X + 1 }>,
    /// Weight but in diagonal for computational advantage
    weights_diag: Matrix<K, { 2 * N_X + 1 }, { 2 * N_X + 1 }>,
}

// Extended Kalman Filter --------------------------------------------------
pub enum EKF_TransitionModel<K: Float, const N_X: usize, const N_Z: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
{
    Linear {
        F: Matrix<K, N_X, N_X>,
        F_transpose: Matrix<K, N_X, N_X>,
        G: Option<Matrix<K, N_X, N_U>>,
    },
    NonLinear {
        f: fn(&Vector<K, N_X>, &Option<Vector<K, N_U>>) -> Vector<K, N_X>,
        jac: fn(&Vector<K, N_X>, &Option<Vector<K, N_U>>) -> Matrix<K, N_X, N_X>,
    },
}

pub enum EKF_ObservationModel<K: Float, const N_X: usize, const N_Z: usize>
where
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
{
    Linear {
        H: Matrix<K, N_Z, N_X>,
        H_transpose: Matrix<K, N_X, N_Z>,
    },
    NonLinear {
        f: fn(&Vector<K, N_X>) -> Vector<K, N_Z>,
        jac: fn(&Vector<K, N_X>) -> Matrix<K, N_Z, N_X>,
    },
}

/// Extended Kalman Filter
pub struct EKF<K: Float, const N_X: usize, const N_Z: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
{
    F: EKF_TransitionModel<K, N_X, N_Z, N_U>,
    H: EKF_ObservationModel<K, N_X, N_Z>,
    I: Matrix<K, N_X, N_X>,
}

// Linear Kalman Filter --------------------------------------------------
/// Linear Kalman Filter
pub struct LKF<K: Float, const N_X: usize, const N_Z: usize, const N_U: usize>
where
    [(); N_X * N_X]:,
    [(); N_X * N_U]:,
    [(); N_Z * N_X]:,
    [(); N_X * N_Z]:,
{
    F: Matrix<K, N_X, N_X>,
    F_transpose: Matrix<K, N_X, N_X>,
    H: Matrix<K, N_Z, N_X>,
    H_transpose: Matrix<K, N_X, N_Z>,
    I: Matrix<K, N_X, N_X>,
    G: Option<Matrix<K, N_X, N_U>>,
}

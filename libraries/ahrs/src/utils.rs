use nalgebra as na;
use core::f32::consts::PI;

/// Convert degrees to radians
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * PI / 180.0
}

/// Convert radians to degrees
pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / PI
}

/// Normalize an angle to the range [-π, π]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut result = angle;
    while result > PI {
        result -= 2.0 * PI;
    }
    while result <= -PI {
        result += 2.0 * PI;
    }
    result
}

/// Calculate the angular difference between two angles in radians
pub fn angle_diff(a: f32, b: f32) -> f32 {
    normalize_angle(a - b)
}

/// Convert a quaternion to Euler angles (roll, pitch, yaw) in radians
pub fn quaternion_to_euler(q: &na::UnitQuaternion<f32>) -> na::Vector3<f32> {
    let euler = q.euler_angles();
    na::Vector3::new(euler.0, euler.1, euler.2)
}

/// Convert Euler angles (roll, pitch, yaw) in radians to a quaternion
pub fn euler_to_quaternion(euler: &na::Vector3<f32>) -> na::UnitQuaternion<f32> {
    na::UnitQuaternion::from_euler_angles(euler.x, euler.y, euler.z)
}

/// Convert a latitude, longitude, altitude to ECEF coordinates
pub fn lla_to_ecef(lat: f32, lon: f32, alt: f32) -> na::Vector3<f32> {
    // WGS84 parameters
    let a = 6378137.0; // semi-major axis
    let f = 1.0 / 298.257_23; // flattening
    let e_squared = 2.0 * f - f * f; // eccentricity squared

    let lat_rad = deg_to_rad(lat);
    let lon_rad = deg_to_rad(lon);

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    let n = a / (1.0 - e_squared * sin_lat * sin_lat).sqrt();

    let x = (n + alt) * cos_lat * cos_lon;
    let y = (n + alt) * cos_lat * sin_lon;
    let z = ((1.0 - e_squared) * n + alt) * sin_lat;

    na::Vector3::new(x, y, z)
}

/// Convert ECEF coordinates to latitude, longitude, altitude
pub fn ecef_to_lla(ecef: &na::Vector3<f32>) -> (f32, f32, f32) {
    // WGS84 parameters
    let a = 6378137.0; // semi-major axis
    let f = 1.0 / 298.257_23; // flattening
    let e_squared = 2.0 * f - f * f; // eccentricity squared

    let x = ecef.x;
    let y = ecef.y;
    let z = ecef.z;

    let p = (x * x + y * y).sqrt();

    // Initial latitude guess
    let mut lat = (z / p).atan();
    let mut alt = 0.0;
    let mut lat_prev;

    // Iterative solution for latitude and altitude
    for _ in 0..5 {
        lat_prev = lat;
        let sin_lat = lat.sin();
        let n = a / (1.0 - e_squared * sin_lat * sin_lat).sqrt();
        alt = p / lat.cos() - n;
        lat = (z / p * (1.0 - e_squared * n / (n + alt))).atan();

        if (lat - lat_prev).abs() < 1e-10 {
            break;
        }
    }

    let lon = y.atan2(x);

    (rad_to_deg(lat), rad_to_deg(lon), alt)
}

/// Convert NED coordinates to ECEF coordinates
pub fn ned_to_ecef(
    ned: &na::Vector3<f32>,
    ref_lat: f32,
    ref_lon: f32,
    ref_alt: f32,
) -> na::Vector3<f32> {
    let ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt);

    let lat_rad = deg_to_rad(ref_lat);
    let lon_rad = deg_to_rad(ref_lon);

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    // Rotation matrix from NED to ECEF
    let r11 = -sin_lat * cos_lon;
    let r12 = -sin_lon;
    let r13 = -cos_lat * cos_lon;

    let r21 = -sin_lat * sin_lon;
    let r22 = cos_lon;
    let r23 = -cos_lat * sin_lon;

    let r31 = cos_lat;
    let r32 = 0.0;
    let r33 = -sin_lat;

    let rotation = na::Matrix3::new(r11, r12, r13, r21, r22, r23, r31, r32, r33);

    ref_ecef + rotation * ned
}

/// Convert ECEF coordinates to NED coordinates
pub fn ecef_to_ned(
    ecef: &na::Vector3<f32>,
    ref_lat: f32,
    ref_lon: f32,
    ref_alt: f32,
) -> na::Vector3<f32> {
    let ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt);

    let lat_rad = deg_to_rad(ref_lat);
    let lon_rad = deg_to_rad(ref_lon);

    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    // Rotation matrix from ECEF to NED
    let r11 = -sin_lat * cos_lon;
    let r12 = -sin_lat * sin_lon;
    let r13 = cos_lat;

    let r21 = -sin_lon;
    let r22 = cos_lon;
    let r23 = 0.0;

    let r31 = -cos_lat * cos_lon;
    let r32 = -cos_lat * sin_lon;
    let r33 = -sin_lat;

    let rotation = na::Matrix3::new(r11, r21, r31, r12, r22, r32, r13, r23, r33);

    rotation * (ecef - ref_ecef)
}

/// Create a skew-symmetric matrix from a 3D vector
pub fn skew_symmetric(v: &na::Vector3<f32>) -> na::Matrix3<f32> {
    na::Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Maximum eigenvalue calculation for a 3x3 matrix
/// Uses power iteration method for better numerical stability
pub fn max_eigenvalue(matrix: &na::Matrix3<f32>) -> f32 {
    // Starting vector for power iteration
    let mut v = na::Vector3::new(1.0, 1.0, 1.0).normalize();

    // Number of iterations
    let max_iter = 10;

    // Perform power iteration
    for _ in 0..max_iter {
        // Apply matrix
        let v_new = matrix * v;

        // Normalize
        if v_new.norm() > 1e-10 {
            v = v_new.normalize();
        } else {
            // If the vector becomes too small, return zero
            return 0.0;
        }
    }

    // Compute Rayleigh quotient
    let rayleigh = (v.transpose() * matrix * v)[0];

    rayleigh.abs()
}

/// Check if a matrix is positive definite
pub fn is_positive_definite(matrix: &na::Matrix3<f32>) -> bool {
    // Compute determinants of leading principal minors
    let d1 = matrix[(0, 0)];
    let d2 = matrix[(0, 0)] * matrix[(1, 1)] - matrix[(0, 1)] * matrix[(1, 0)];
    let d3 = matrix.determinant();

    // Matrix is positive definite if all determinants are positive
    d1 > 0.0 && d2 > 0.0 && d3 > 0.0
}

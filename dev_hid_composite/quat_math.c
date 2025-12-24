#include <math.h>
#include <stdio.h>

typedef struct {
    float w, x, y, z;
} quat_t;

typedef struct {
    float x, y, z;
} vect_t;

// -------- Vector functions --------

static inline vect_t vect(float x, float y, float z) {
    vect_t v = {x, y, z};
    return v;
}

static inline float vect_dot(vect_t a, vect_t b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline vect_t vect_cross(vect_t a, vect_t b) {
    return vect(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}

// ||v||
static inline float vect_norm(vect_t v) {
    return sqrtf(vect_dot(v, v));
}

static inline vect_t vect_normalize(vect_t v) {
    float n = vect_norm(v);
    if (n < 1e-6f) return vect(0.0f, 0.0f, 0.0f); // avoid division by zero
    float inv = 1.0f / n; // make intermediate variable to avoid repeated division
    return vect(v.x * inv, v.y * inv, v.z * inv);
}

// -------- Quaternion functions --------

static inline quat_t quat_identity(void) {
    quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

static inline quat_t quat(float w, float x, float y, float z) {
    quat_t q = {w, x, y, z};
    return q;
}

static inline quat_t quat_normalize(quat_t q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-6f) return quat_identity(); // avoid division by zero
    float inv = 1.0f / n; // make intermediate variable to avoid repeated division
    return quat(q.w*inv, q.x*inv, q.y*inv, q.z*inv);
}

// q = q1 * q2
static inline quat_t quat_mul(quat_t q1, quat_t q2) {
    return quat(
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    );
}

static inline quat_t quat_conj(quat_t q) {
    return quat(q.w, -q.x, -q.y, -q.z);
}

// Axis must be unit-length, angle in radians
static inline quat_t quat_from_axis_angle(float angle, vect_t axis) {
    float half = 0.5f * angle;
    float s = sinf(half);
    return quat(cosf(half), axis.x * s, axis.y * s, axis.z * s);
}

// Rotate vector v by quaternion q: v' = q * qv * q^{-1}
static inline vect_t quat_rotate_vec(quat_t q, vect_t v) {
    quat_t qv = {0.0f, v.x, v.y, v.z};
    quat_t q_conj = quat_conj(q);
    quat_t rot = quat_mul(quat_mul(q, qv), q_conj);
    return vect(rot.x, rot.y, rot.z);
}

////////////////////////////////FUNCTIONS////////////////////////////////

static quat_t q_est = {1.0f, 0.0f, 0.0f, 0.0f}; //current orientation estimate
static float alpha = 0.98f; //complementary filter coefficient

// Reset orientation
void imu_filter_reset(void) {
    q_est = quat_identity();
}

// Get current orientation estimate
quat_t imu_get_quaternion(void) {
    return q_est;
}

void imu_set_alpha(float new_alpha) {
    if (new_alpha < 0.0f) new_alpha = 0.0f;
    if (new_alpha > 1.0f) new_alpha = 1.0f;
    alpha = new_alpha;
}


void imu_update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt)
{
    // 1) Integrate gyro to get predicted orientation q_w

    // Angular velocity vector
    vect_t w = vect(gx, gy, gz);
    float w_mag = vect_norm(w); //TODO: needed? is the if needed?

    quat_t q_w = q_est;

    if (w_mag > 1e-6f) {
        // Axis of rotation (unit vector) -> w/|w|
        vect_t axis = vect(w.x / w_mag, w.y / w_mag, w.z / w_mag);

        // Angle of rotation (radians) -> |w| * dt
        float angle = w_mag * dt;

        quat_t q_delta = quat_from_axis_angle(angle, axis); // eq(17)
        q_w = quat_mul(q_est, q_delta); // eq(18)
        q_w = quat_normalize(q_w);
    }

    // 2) Use accelerometer to compute tilt (gravity direction)

    // Normalize accel to get direction of "down" in body frame
    vect_t a_body = vect(ax, ay, az);
    vect_t a_body_norm = vect_normalize(a_body);

    // TODO: check this
    if (vect_norm(a_body_norm) < 1e-3f) {
        // Accel too small / invalid; skip correction
        q_est = q_w;
        return;
    }

    // Rotate accel into world frame using gyro-only orientation
    vect_t q_a_world = quat_rotate_vec(q_w, a_body_norm); // already normalized -> (Vx, Vy, Vz)


    // Desired direction of gravity in world frame: (0, 1, 0)
    vect_t g_world = vect(0.0f, 0.0f, 1.0f);

    // Compute axis and angle to rotate q_a_world onto g_world
    //vect_t norm_correction_axis = vect_normalize(vect_cross(q_a_world, g_world)); // n/||n||
    //float axis_norm = vect_norm(correction_axis); // ||n||

    // If already aligned (or nearly), no correction needed TODO: could cause problems with 180 flip: remove unless needed
    // if (vect_norm(correction_axis) < 1e-6f) {
    //     q_est = q_w;
    //     return;
    // }

    // correction_axis.x /= axis_norm;
    // correction_axis.y /= axis_norm;
    // correction_axis.z /= axis_norm;

    
    // shortcut from notes TODO: might change based on setup
    //vect_t norm_correction_axis = vect_normalize(vect(-q_a_world.z, 0, q_a_world.x)); //  n/||n||

    // Angle between vectors: cos(phi) = dot(q_a_world, g_world)
    //TODO: might need to change back to float cos_phi = vect_dot(q_a_world, g_world);
    float cos_phi = vect_dot(q_a_world, g_world);
    if (cos_phi > 1.0f) cos_phi = 1.0f; // clamp TODO: needed?
    if (cos_phi < -1.0f) cos_phi = -1.0f;

    // SINGULARITY CHECK:
    // If vectors are very close to parallel (cos_phi ~ 1) or anti-parallel (cos_phi ~ -1),
    // we cannot calculate a stable rotation axis.
    if (fabsf(cos_phi) > 0.999f) {
        // Already aligned, or 180 degrees flipped. 
        // In 180 flip case, we can't safely correct without a magnetometer
        q_est = q_w;
        return;
    }

    float phi = acosf(cos_phi); //eq(23)

    // 3) Complementary filter: apply only a fraction (1 - alpha) of this correction
    float phi_corr = (1.0f - alpha) * phi;

    // Compute axis and angle to rotate q_a_world onto g_world
    vect_t norm_correction_axis = vect_normalize(vect_cross(q_a_world, g_world)); // n/||n||

    quat_t q_tilt = quat_from_axis_angle(phi_corr, norm_correction_axis); //eq (24)

    // Apply tilt correction in world frame:
    // new orientation = q_tilt * q_w
    quat_t q_new = quat_mul(q_tilt, q_w);
    q_est = quat_normalize(q_new);

    //vec3_t v_world = quat_rotate_vec(q_est, v_body);
}
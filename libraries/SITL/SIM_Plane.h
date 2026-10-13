/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_JSON/AP_JSON.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Plane(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    float angle_of_attack;
    float beta;

    const struct Coefficients {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 14.58;            // Powierzchnia nośna S
        float b = 8.88;             // Rozpiętość b
        float c = 1.76;             // Średnia cięciwa aerodynamiczna Ca
       float c_lift_0 = 0.00063;   // -Aero_Cz_Base dla alpha = 0 st.
        float c_lift_deltae = 0.4578; // -Cz_dh
        float c_lift_a = 4.5627;    // Gradient d(Cl)/d(alpha) pomiędzy -1 a 1 st.
        float c_lift_q = 7.1187;    // -Cz_dq
        float mcoeff = 50;          // Pozostawiono domyślne
        float oswald = 0.9;         // Pozostawiono domyślne
        float alpha_stall = 0.4712; // Pozostawiono domyślne
        float c_drag_q = -0.0589;   // -Cx_dq (w układzie body drag to -X)
        float c_drag_deltae = 0.0;  // Brak danych w modelu
        float c_drag_p = 0.1;       // Pozostawiono domyślne
        float c_y_0 = 0;            // Brak danych bazowych dla C_y_0
        float c_y_b = -0.2545;      // CydBeta dla alpha = 0 st.
        float c_y_p = 0.0225;       // CydP
        float c_y_r = 0.2228;       // CydR
        float c_y_deltaa = 0;       // Brak danych w modelu
        float c_y_deltar = 0.1723;  // CydDv
        float c_l_0 = 0;            // Brak danych bazowych dla C_l_0
        float c_l_p = -0.3795;      // CldP
        float c_l_b = -0.0162;      // CldBeta dla alpha = 0 st.
        float c_l_r = 0.0088;       // CldR
        float c_l_deltaa = -0.2234; // CldDl
        float c_l_deltar = 0.0118;  // CldDv
        float c_m_0 = -0.0050;      // Aero_Cm_Base dla alpha = 0 st.
        float c_m_a = -0.2392;      // Gradient d(Cm)/d(alpha) pomiędzy -1 a 1 st.
        float c_m_q = -8.7749;      // Cm_dq
        float c_m_deltae = -1.3747; // Cm_dh
        float c_n_0 = 0;            // Brak danych bazowych dla C_n_0
        float c_n_b = 0.0642;       // CndBeta dla alpha = 0 st.
        float c_n_p = -0.0129;      // CndP
        float c_n_r = -0.0972;      // CndR
        float c_n_deltaa = 0.0046;  // CndDl
        float c_n_deltar = -0.1174; // CndDv
        float deltaa_max = 0.3491;  // Pozostawiono domyślne
        float deltae_max = 0.3491;  // Pozostawiono domyślne
        float deltar_max = 0.3491;  // Pozostawiono domyślne
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{0, 0, 0};
    } default_coefficients;

    struct Coefficients coefficient;

    float thrust_scale;
    bool reverse_thrust;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool redundant;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool aerobatic;
    bool copter_tailsitter;
    bool have_launcher;
    bool have_steering;
    float launch_accel;
    float launch_time;
    uint64_t launch_start_ms;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate,
        true
    };

    // load aero coefficients from a json model file
    void load_coeffs(const char *model_json);
    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);

private:
    // json parsing helpers (TODO reduce code duplication)
    void parse_float(AP_JSON::value val, const char* label, float &param);
    void parse_vector3(AP_JSON::value val, const char* label, Vector3f &param);

    void update_battery(const struct sitl_input &input) override;
    // Plane needs the sitl_input, should never use the parent's input-free version.
    // (Using it would be using the wrong battery model, but it would silently work.)
    void update_battery() override { INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control); };
    bool battery_is_empty() { return battery_voltage < 0.5f; };
};

} // namespace SITL

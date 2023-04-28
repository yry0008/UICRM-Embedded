#pragma once

#include "bsp_can.h"
#include "bsp_pwm.h"
#include "controller.h"
#include "utils.h"

namespace control {

    constexpr int MOTOR_RANGE = 30000;  // TODO: 32767 or 30000?

    int16_t ClipMotorRange(float output);

    /**
     * @brief base class for motor representation
     */
    class MotorBase {
      public:
        MotorBase() : output_(0) {
        }
        virtual ~MotorBase() {
        }

        virtual void SetOutput(int16_t val) {
            output_ = val;
        }

      protected:
        int16_t output_;
    };

    /**
     * @brief base class for CAN motor representation
     */
    class MotorCANBase : public MotorBase {
      public:
        /**
         * @brief base constructor
         *
         * @param can    CAN instance
         * @param rx_id  CAN rx id
         */
        MotorCANBase(bsp::CAN* can, uint16_t rx_id);

        /**
         * @brief update motor feedback data
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        virtual void UpdateData(const uint8_t data[]) = 0;

        /**
         * @brief print out motor data
         */
        virtual void PrintData() const = 0;

        /**
         * @brief get motor angle, in [rad]
         *
         * @brief 获得电机的角度，格式为[rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        virtual float GetTheta() const;

        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @brief 获得电机的角度差，格式为[rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        virtual float GetThetaDelta(const float target) const;

        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @brief 获得电机的角速度，格式为[rad / s]
         *
         * @return angular velocity
         */
        virtual float GetOmega() const;

        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @brief 获得电机的角速度差，格式为[rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        virtual float GetOmegaDelta(const float target) const;

        virtual int16_t GetCurr() const;

        virtual uint16_t GetTemp() const;

        /**
         * @brief transmit CAN message for setting motor outputs
         *
         * @param motors[]    array of CAN motor pointers
         * @param num_motors  number of motors to transmit
         */
        static void TransmitOutput(MotorCANBase* motors[], uint8_t num_motors);

        /**
         * @brief set ServoMotor as friend of MotorCANBase since they need to use
         *        many of the private parameters of MotorCANBase.
         */
        friend class ServoMotor;

        volatile bool connection_flag_ = false;

      protected:
        volatile float theta_;
        volatile float omega_;

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
    };

    /**
     * @brief DJI 2006 motor class
     */
    class Motor2006 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor2006(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
    };

    /**
     * @brief DJI 3508 motor class
     */
    class Motor3508 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor3508(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
    };

    /**
     * @brief DJI 6020 motor class
     */
    class Motor6020 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6020(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;

        int16_t GetCurr() const override final;

        uint16_t GetTemp() const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile uint8_t raw_temperature_ = 0;
    };

    /**
     * @brief DJI 6623 motor class
     */
    class Motor6623 : public MotorCANBase {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6623(bsp::CAN* can, uint16_t rx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
        /* implements data printout */
        void PrintData() const override final;
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
        /* override default implementation with not implemented */
        float GetOmega() const override final;
        float GetOmegaDelta(const float target) const override final;

      private:
        volatile int16_t raw_current_get_ = 0;
        volatile int16_t raw_current_set_ = 0;

        static const int16_t CURRENT_CORRECTION = -1;  // current direction is reversed
    };

    /**
     * @brief PWM motor base class
     */
    class MotorPWMBase : public MotorBase {
      public:
        /**
         * @brief constructor
         *
         * @param htim           HAL timer handle
         * @param channel        HAL timer channel, from [0, 4)
         * @param clock_freq     clock frequency associated with the timer, in [Hz]
         * @param output_freq    desired output frequency, in [Hz]
         * @param idle_throttle  idling pulse width, in [us]
         *
         * @note M3508 have idle_throttle about 1500, snail have idle_throttle about
         * 1100
         */
        MotorPWMBase(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq,
                     uint32_t output_freq, uint32_t idle_throttle);

        /**
         * @brief set and transmit output
         *
         * @param val offset value with respect to the idle throttle pulse width, in
         * [us]
         */
        virtual void SetOutput(int16_t val) override;

      private:
        bsp::PWM pwm_;
        uint32_t idle_throttle_;
    };

    /**
     * @brief DJI snail 2305 motor class
     */
    class Motor2305 : public MotorPWMBase {
      public:
        /* override base implementation with max current protection */
        void SetOutput(int16_t val) override final;
    };

    /**
     * @brief servomotor turning mode
     * @note the turning direction is determined as if user is facing the motor,
     * may subject to change depending on motor type
     */
    typedef enum {
        SERVO_CLOCKWISE = -1,   /* Servomotor always turn clockwisely */
        SERVO_NEAREST = 0,      /* Servomotor turn in direction that make movement minimum */
        SERVO_ANTICLOCKWISE = 1 /* Servomotor always turn anticlockwisely */
    } servo_mode_t;

    /**
     * @brief servomotor status
     * @note the turning direction is determined as if user is facing the motor,
     * may subject to change depending on motor type
     */
    typedef enum {
        TURNING_CLOCKWISE = -1,   /* Servomotor is turning clockwisely         */
        INPUT_REJECT = 0,         /* Servomotor rejecting current target input */
        TURNING_ANTICLOCKWISE = 1 /* Servomotor is turning anticlockwisely     */
    } servo_status_t;

/**
 * @brief transmission ratios of DJI motors, reference to motor manuals for
 * more details
 */
#define M3508P19_RATIO (3591.0 / 187) /* Transmission ratio of M3508P19 */
#define M2006P36_RATIO 36             /* Transmission ratio of M2006P36 */

    typedef struct {
        servo_mode_t mode; /* turning mode of servomotor, refer to type servo_mode_t */
        float speed;       /* motor shaft turning speed                              */
    } servo_jam_t;

    class ServoMotor;  // declare first for jam_callback_t to have correct param
                       // type
    /**
     * @brief jam callback template
     */
    typedef void (*jam_callback_t)(ServoMotor* servo, const servo_jam_t data);

    /**
     * @brief structure used when servomotor instance is initialized
     */
    typedef struct {
        MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
        float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor */
        float* omega_pid_param;   /* pid parameter used to control speed of motor   */
        float max_iout;
        float max_out;
        float* hold_pid_param;
        float hold_max_iout;
        float hold_max_out;
    } servo_t;

    /**
     * @brief wrapper class for motor to enable the motor shaft angle to be
     * precisely controlled with possible external gearbox present
     * @note this is a calculation class that calculate the motor output for
     * desired output, but it does not directly command a motor to turn.
     */
    class ServoMotor {
      public:
        /**
         * @brief base constructor
         *
         * @param servo         initialization struct, refer to type servo_t
         * @param proximity_in  critical difference angle for the motor to enter hold
         * state
         * @param proximity_out critical difference angle for the motor to exit hold
         * state
         *
         * @note proximity_out should be greater than proximity_in
         */
        ServoMotor(servo_t data, float align_angle = -1, float proximity_in = 0.05,
                   float proximity_out = 0.15);

        /**
         * @brief set next target for servomotor, will have no effect if last set
         * target has not been achieved
         * @note if motor is not holding, call to this function will have no effect
         * unless override is true
         *
         * @param target   next target for the motor in [rad]
         * @param override if true, override current target even if motor is not
         * holding right now
         * @return current turning mode of motor
         */
        servo_status_t SetTarget(const float target, bool override = false);

        /**
         * @brief set turning speed of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_speed speed of desired motor shaft turning speed, in [rad/s]
         */
        void SetMaxSpeed(const float max_speed);

        /**
         * @brief set acceleration of motor when moving
         *
         * @note should always be positive, negative inputs will be ignored
         *
         * @param max_acceleration speed of desired motor shaft turning speed, in
         * [rad/s]
         */
        void SetMaxAcceleration(const float max_acceleration);

        /**
         * @brief calculate the output of the motors under current configuration
         * @note this function will not command the motor, it only calculate the
         * desired input
         */
        void CalcOutput();

        /**
         * @brief if the motor is holding
         *
         * @return true  the motor is holding (i.e. not turning)
         * @return false the motor is not holding (i.e. turning)
         */
        bool Holding() const;

        /**
         * @brief get current servomotor target, in [rad]
         *
         * @return current target angle, range between [0, 2PI]
         */
        float GetTarget() const;

        /**
         * @brief register a callback function that would be called if motor is
         * jammed
         * @note Jam detection uses a moving window across inputs to the motor. It
         * uses a circular buffer of size detect_period to store history inputs and
         * calculates a rolling average of the inputs. Everytime the average of
         * inputs is greater than effect_threshold * 32768(maximum command a motor
         * can accept), the jam callback function will be triggered once. The
         * callback will only be triggered once each time the rolling average cross
         * the threshold from lower to higher. For a standard jam callback function,
         * refer to example motor_m3508_antijam
         *
         * @param callback         callback function to be registered
         * @param effort_threshold threshold for motor to be determined as jammed,
         * ranged between (0, 1)
         * @param detect_period    detection window length
         */
        void RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                 uint8_t detect_period = 50);

        /**
         * @brief print out motor data
         */
        void PrintData() const;

        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle, range between [0, 2PI]
         */
        float GetTheta() const;

        /**
         * @brief get angle difference (target - actual), in [rad]
         *
         * @param target  target angle, in [rad]
         *
         * @return angle difference, range between [-PI, PI]
         */
        float GetThetaDelta(const float target) const;

        /**
         * @brief get angular velocity, in [rad / s]
         *
         * @return angular velocity
         */
        float GetOmega() const;

        /**
         * @brief get angular velocity difference (target - actual), in [rad / s]
         *
         * @param target  target angular velocity, in [rad / s]
         *
         * @return difference angular velocity
         */
        float GetOmegaDelta(const float target) const;

        /**
         * @brief update the current theta for the servomotor
         * @note only used in CAN callback, do not call elsewhere
         *
         * @param data[]  raw data bytes
         */
        void UpdateData(const uint8_t data[]);

        friend class SteeringMotor;

      private:
        // refer to servo_t for details
        MotorCANBase* motor_;
        float max_speed_;
        float max_acceleration_;
        float transmission_ratio_;
        float proximity_in_;
        float proximity_out_;

        // angle control
        bool hold_; /* true if motor is holding now, otherwise moving now */
        uint64_t start_time_;
        float target_angle_; /* desired target angle, range between [0, 2PI] in [rad]
                              */
        float align_angle_;  /* motor angle when a instance of this class is created
                                with that motor    */
        float motor_angle_;  /* current motor angle in [rad], with align_angle
                                subtracted               */
        float offset_angle_; /* cumulative offset angle of motor shaft, range between
                                [0, 2PI] in [rad] */
        float servo_angle_;  /* current angle of motor shaft, range between [0, 2PI]
                                in  [rad]           */
        float cumulated_angle_;

        // jam detection
        jam_callback_t jam_callback_; /* callback function that will be invoked if
                                         motor jammed */
        int detect_head_;             /* circular buffer current head             */
        int detect_period_;           /* circular buffer length           */
        int detect_total_;            /* rolling sum of motor inputs            */
        int jam_threshold_;           /* threshold for rolling sum for the motor to be
                                         considered as jammed */
        int16_t* detect_buf_;         /* circular buffer         */

        // pid controllers
        ConstrainedPID omega_pid_; /* pid for controlling speed of motor */
        ConstrainedPID hold_pid_;

        // edge detectors
        FloatEdgeDetector* inner_wrap_detector_; /* detect motor motion across encoder boarder */
        FloatEdgeDetector* outer_wrap_detector_; /* detect motor motion across encoder boarder */
        BoolEdgeDetector* hold_detector_;        /* detect motor is in mode toggling, reset
                                                    pid accordingly  */
        BoolEdgeDetector* jam_detector_;         /* detect motor jam toggling, call jam
                                                    callback accordingly */
    };

    typedef bool (*align_detect_t)(void);

    /**
     * @brief structure used when steering motor instance is initialized
     */
    typedef struct {
        MotorCANBase* motor; /* motor instance to be wrapped as a servomotor      */
        float max_speed;     /* desired turning speed of motor shaft, in [rad/s]  */
        float test_speed;
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor */
        float offset_angle;
        float* omega_pid_param; /* pid parameter used to control speed of motor */
        float max_iout;
        float max_out;
        align_detect_t align_detect_func;
        float calibrate_offset;
    } steering_t;

    class SteeringMotor {
      public:
        SteeringMotor(steering_t data);
        float GetRawTheta() const;
        /**
         * @brief print out motor data
         */
        void PrintData() const;
        void TurnRelative(float angle);
        void TurnAbsolute(float angle);
        bool AlignUpdate();
        void Update();

      private:
        ServoMotor* servo_;

        float test_speed_;
        align_detect_t align_detect_func;
        float calibrate_offset;

        float align_angle_;
        BoolEdgeDetector* align_detector;
        bool align_complete_;
    };

    typedef enum {
        MIT = 0,
        POS_VEL = 1,
        VEL = 2,
    } m4310_mode_t;
    /**
     * @brief m4310 motor class
     */
    class Motor4310 {
      public:
        /** constructor wrapper over MotorCANBase
         *  CAN frame id for different modes:
         *      MIT:                  actual CAN id.
         *      position-velocity:    CAN id + 0x100.
         *      velocity:             CAN id + 0x200.
         *  @param can    CAN object
         *  @param rx_id  Master id
         *  @param tx_id  CAN id *** NOT the actual CAN id but the id configured in software ***
         *  @param mode   0: MIT
         *                1: position-velocity
         *                2: velocity
         */
        Motor4310(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, m4310_mode_t mode);

        /* implements data update callback */
        void UpdateData(const uint8_t data[]);

        /* enable m4310 */
        void MotorEnable();
        /* disable m4310 */
        void MotorDisable();
        /* set zero position */
        void SetZeroPos();

        /**
         * @brief get motor angle, in [rad]
         *
         * @return radian angle
         */
        float GetTheta() const;
        /**
         * @brief get motor angular velocity, in [rad / s]
         * @return radian angular velocity
         */
        float GetOmega() const;

        /**
         * @brief get motor torque, in [Nm]
         * @return motor torque
         */
        float GetTorque() const;
        /**
         * implements transmit output specifically for 4310
         * @param motor m4310 motor object
         * @param mode operation modes:
         *                0: MIT
         *                1: position-velocity
         *                2: velocity
         */
        void TransmitOutput();

        /* implements data printout */
        void PrintData();

        /* set output parameters for m4310 using MIT mode */
        void SetOutput(float position, float velocity, float kp, float kd, float torque);

        /* set output parameters for m4310 using position-velocity mode */
        void SetOutput(float position, float velocity);

        /* set output parameters for m4310 using velocity mode */
        void SetOutput(float velocity);

        /**
         * @brief Converts a float to an unsigned int, given range and number of bits;
         *      see m4310 V1.2 document for detail
         * @param x value to be converted
         * @param x_min minimum value of the current parameter
         * @param x_max maximum value of the current parameter
         * @param bits size in bits
         * @return value converted from float to unsigned int
         */
        static int16_t float_to_uint(float x, float x_min, float x_max, int bits);
        /**
         * @brief Converts an unsigned int to a float, given range and number of bits;
         *      see m4310 V1.2 document for detail
         * @param x_int value to be converted
         * @param x_min minimum value of the current parameter
         * @param x_max maximum value of the current parameter
         * @param bits size in bits
         * @return value converted from float to unsigned int
         */
        static float uint_to_float(int x_int, float x_min, float x_max, int bits);

        volatile bool connection_flag_ = false;

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
        uint16_t tx_id_actual_;

        volatile m4310_mode_t mode_;     // current motor mode
        volatile float kp_set_ = 0;      // defined kp value
        volatile float kd_set_ = 0;      // defined kd value
        volatile float vel_set_ = 0;     // defined velocity
        volatile float pos_set_ = 0;     // defined position
        volatile float torque_set_ = 0;  // defined torque

        volatile int16_t raw_pos_ = 0;        // actual position
        volatile int16_t raw_vel_ = 0;        // actual velocity
        volatile int16_t raw_torque_ = 0;     // actual torque
        volatile int16_t raw_motorTemp_ = 0;  // motor temp
        volatile int16_t raw_mosTemp_ = 0;    // MOS temp
        volatile float theta_ = 0;            // actual angular position
        volatile float omega_ = 0;            // actual angular velocity
        volatile float torque_ = 0;           // actual torque

        constexpr static float KP_MIN = 0;
        constexpr static float KP_MAX = 500;
        constexpr static float KD_MIN = 0;
        constexpr static float KD_MAX = 5;
        constexpr static float P_MIN = -12.5;
        constexpr static float P_MAX = 12.5;
        constexpr static float V_MIN = -45;
        constexpr static float V_MAX = 45;
        constexpr static float T_MIN = -18;
        constexpr static float T_MAX = 18;
    };

} /* namespace control */

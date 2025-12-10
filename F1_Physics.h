#ifndef F1_PHYSICS_H
#define F1_PHYSICS_H

#include <vector>
#include <array>
#include <cmath>

class F1PhysicsEngine
{
public:
    struct Point2D
    {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
    };

    struct Vector2D
    {
        double x, y;
        Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    };

    struct CarState
    {
        // Поступательное движение
        Point2D position = {1869.0, 1160.0};
        Vector2D velocity;
        Vector2D acceleration;
        Vector2D world_velocity;
        double speed;
        double wheel_speed;
        double wheel_acceleration;
        double linear_slip;
        double slip_coef = 10;

    

        // Вращательное движение
        double car_angle = 0.0;
        double angular_velocity = 0.0;
        double steering_wheel = 0.0;
        double slip_angle = 0.0;
        // Трение
        bool is_Skeeding = false;
        double current_tire_friction = 1.5;
        double current_tire_temperature = 70;
        double slip_ratio = 0.0;
        double current_tire_wear =0.0;


        // Двигатель и трансмиссия
	double max_rpm = 10000;
	double max_torque = 300;
	double acceleration_rate = 200;
	double rotor_resist = 100;
	double last_engine_rpm = 0.0;
        double engine_rpm = 0.0;	
        double engine_torque = 0.0;
        double wheel_rpm = 0.0;
	double last_wheel_rpm = 0.0;
        double wheel_torque = 0.0;
        int current_gear = 1;
	int previous_gear = 1;
        double wheel_based_speed = 0.0;
        double wheel_torque_final = 0.0;

        // Силы
        double traction_force = 0.0;
        double drag_force = 0.0;
        double brake_force = 0.0;
        double down_force = 0.0;
        double long_force = 0.0;
        double side_force = 0.0;

        // Тормозная система
        double brake_factor = 0.0;
    };

private:
    CarState current_state;

    struct CarParameters
    {
        // Геометрия
        double wheel_radius = 0.33;
        double mass = 540.0;
        double wheelbase = 3.2;  // База автомобиля
        double moment_arm = 1.6; // Плечо момента
        double inertia = 500.0; // Момент инерции

        // Двигатель и трансмиссия
        double max_rpm = 10000.0;
        double rotor_resist_coef = 10.0; 
        double rotor_inertia = 200.0;
	double max_limit_torque = 1000;
	double min_limit_torque = 100;
	double dtorque = 300;
        double peak_rpm = 8000.0;                                                  
        double accel_coef = 20.0;                                          
        std::array<double, 4> gear_ratios = {0.0, 2.2, 1.5, 1.1};
        double final_drive = 5.0;

        // Аэродинамика
        double drag_coefficient = 0.5; // Увеличенное сопротивление
        double frontal_area = 1.0;
        double air_density = 0.9;
        double downforce_coefficient = 5.0;

        // Шины и тормоза
        std::vector<double> tire_friction = {1.5, 0.2}; // Улучшенное сцепление
        double max_brake_force = 35000.0;
        double brake_factor_k = 0.2;
	double brake_max = 1.0;
	double brake_min = 0.0;
        double brake_rate = 1000.0;
        double cornering_stiffness = 12000.0; // Жесткость в повороте
        double steering_speed = 1.5;
        double lateral_damping_coef = 10;
        double rotational_damping = 5.0; // Демпфирование вращения
        double max_steering_wheel = M_PI;
        double wheel_inertia = 0.45;
        double A_slip_ratio = 10;
        double A_slip_angle = 100.0;
        double K_temp = 0.0015;
        double temp_outside = 25;
        double B_slip_ratio = 0.3;
        double B_slip_angle = 0.50;
        double K_wear    = 0.05;
        double temp_optimal = 100;
    };

    CarParameters params;

public:
    F1PhysicsEngine();
    void reset();
    void update(double dt, bool gas_pedal, bool brake_pedal, double steering_wheel = 0.0);
    void shiftUp();
    void shiftDown();
    void gas_up();
    void gas_down();
    void brake_up();
    void brake_down();
    const CarState &getState() const { return current_state; }

private:
    void calculateEnginePhysics(bool gas_pedal, double dt);
    void calculateRPM(bool gas_pedal, double dt);
    void calculateTorque();
    void calculateWheelParameters();
    void calculateBrakeFactor(bool brake_pedal, double dt);
    void applyBrakes(double dt);
    void TestMaxRPMForCurrentGear();
    void acceleration_rate(double rpm);

    void calculate_Long_Forces(bool gas_pedal, bool brake_pedal);
    void calculate_Side_Forces(double steering, double dt);
    void calculateDragForce();
    void calculateDownForce();
    void calculateTractionForce(bool gas_pedal);
    void calculateBrakeForce();

    void IsSkeeding(bool gas_pedal);
    void integrateMotion(double dt);
    void integrateWheelMotion(double dt);
    void CalculateTireParams(double dt);

    void save_to_table(double);
};

#endif // F1_PHYSICS_H

#include "F1_Physics.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <vector>
#include <iomanip>

using namespace std;

double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

double normalize_steer(double angle)
{
    while (angle > M_PI)
        angle = M_PI;
    while (angle < -M_PI)
        angle = -M_PI;
    return angle;
}

// Конструктор
F1PhysicsEngine::F1PhysicsEngine()
{
    reset();
}

void F1PhysicsEngine::reset()
{
  	  current_state = CarState();
   	 current_state.current_gear = 1;
        current_state.current_tire_temperature = 70;
	

}

// Основной метод обновления
void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering_input)
{
    // 1. Обновление угла руля
    double target_steering = steering_input * params.max_steering_wheel;
    double steering_diff = target_steering - current_state.steering_wheel;

    // Плавное изменение руля
    if (fabs(steering_diff) > 0.001)
    {
        double max_steering_change = params.steering_speed * dt;
        if (fabs(steering_diff) > max_steering_change)
        {
            steering_diff = (steering_diff > 0) ? max_steering_change : -max_steering_change;
        }
        current_state.steering_wheel += steering_diff;
    }

    calculateEnginePhysics(gas_pedal, dt);
    calculate_Side_Forces(current_state.steering_wheel, dt);
    calculate_Long_Forces(gas_pedal, brake_pedal);
    IsSkeeding(gas_pedal);
   // calculate_Long_Forces(gas_pedal, brake_pedal);
    integrateMotion(dt);
    integrateWheelMotion(dt);
    CalculateTireParams(dt);
    save_to_table(dt);
}

// Управление передачами
void F1PhysicsEngine::shiftUp()
{
    if ((current_state.current_gear < 3) && (current_state.current_gear > 0))
    {
        current_state.previous_gear = current_state.current_gear;
        current_state.current_gear++;
        current_state.engine_rpm = current_state.wheel_rpm * params.final_drive * params.gear_ratios[current_state.current_gear];
    }
    else if (current_state.current_gear == 0)
    {
        current_state.previous_gear = current_state.current_gear;
        current_state.current_gear++;
        current_state.engine_rpm = current_state.wheel_rpm * params.final_drive * params.gear_ratios[current_state.current_gear];
    }

    acceleration_rate(current_state.engine_rpm);
}

void F1PhysicsEngine::gas_up() {
	current_state.max_torque += params.dtorque;
	if (current_state.max_torque > params.max_limit_torque) {
		current_state.max_torque = params.max_limit_torque;
	}
}

void F1PhysicsEngine::gas_down() {
	current_state.max_torque -= params.dtorque;
	if (current_state.max_torque < params.min_limit_torque) {
		current_state.max_torque = params.min_limit_torque;
	}
}

void F1PhysicsEngine::brake_up() {
	current_state.brake_factor += params.brake_factor_k;
	if (current_state.brake_factor > params.brake_max) {
		current_state.brake_factor = params.brake_max;
	}
}

void F1PhysicsEngine::brake_down() {
	current_state.brake_factor -= params.brake_factor_k;
	if (current_state.brake_factor < params.brake_min) {
		current_state.brake_factor = params.brake_min;
	}
}

void F1PhysicsEngine::shiftDown()
{
    if (current_state.current_gear > 1)
    {
        double new_gear_factor = params.final_drive * params.gear_ratios[current_state.current_gear - 1];

        if (current_state.wheel_rpm * new_gear_factor <= params.max_rpm)
        {
            current_state.previous_gear = current_state.current_gear;
            current_state.current_gear--;
            current_state.engine_rpm = current_state.wheel_rpm * params.final_drive * params.gear_ratios[current_state.current_gear];
        }
    }
    else if (current_state.current_gear == 1)
    {
        current_state.previous_gear = current_state.current_gear;
        current_state.current_gear = 0;
    }

    acceleration_rate(current_state.engine_rpm);
}

// Двигатель и трансмиссия
void F1PhysicsEngine::calculateEnginePhysics(bool gas_pedal, double dt)
{
    calculateRPM(gas_pedal, dt);
    // TestMaxRPMForCurrentGear();
    calculateTorque();
    calculateWheelParameters();
}

void F1PhysicsEngine::acceleration_rate(double rpm)
{
    current_state.acceleration_rate = 200 + fabs(rpm) * params.accel_coef;
    current_state.rotor_resist = 100 + fabs(rpm) * params.rotor_resist_coef;
}

void F1PhysicsEngine::calculateRPM(bool gas_pedal, double dt)
{
    double mean_wheel_load = (params.mass + current_state.down_force / 9.81);
    double new_engine_rpm;
    double gear_factor;

    if (current_state.current_gear != 0)
    {
        gear_factor = params.gear_ratios[current_state.current_gear] * params.final_drive;
        new_engine_rpm = current_state.wheel_rpm * gear_factor;

        if (gas_pedal)
        {
            current_state.engine_rpm = new_engine_rpm + mean_wheel_load * pow(params.wheel_radius, 2) / params.rotor_inertia * (-current_state.engine_rpm + new_engine_rpm);

            current_state.engine_rpm += dt * current_state.acceleration_rate - dt * current_state.rotor_resist;

            if (current_state.engine_rpm < 0)
            {
                current_state.engine_rpm = 0;
            }
        }
        else
        {
            current_state.engine_rpm += mean_wheel_load * pow(params.wheel_radius, 2) / params.rotor_inertia * (-current_state.engine_rpm + new_engine_rpm);

            current_state.engine_rpm -= dt * current_state.rotor_resist;

            if (current_state.engine_rpm < 0)
            {
                current_state.engine_rpm = 0;
            }
        }
    }
    else
    {
        current_state.engine_rpm -= current_state.rotor_resist * dt;
        if (gas_pedal)
        {
            current_state.engine_rpm += dt * current_state.acceleration_rate;
        }

        if (current_state.engine_rpm < 0)
        {
            current_state.engine_rpm = 0;
        }
    }

    if (current_state.engine_rpm > params.max_rpm)
    {
        current_state.engine_rpm = params.max_rpm;
    }

    acceleration_rate(current_state.engine_rpm);
}

void F1PhysicsEngine::TestMaxRPMForCurrentGear()
{
    if (current_state.current_gear > 0)
    {
        double gear_ratio = params.gear_ratios[current_state.current_gear];
        double gear_factor = gear_ratio * params.final_drive;
        double max_wheel_rpm = params.max_rpm / gear_factor;

        if (current_state.wheel_rpm > max_wheel_rpm)
        {
            current_state.wheel_rpm = max_wheel_rpm;
            // current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
        }
    }
}

void F1PhysicsEngine::calculateTorque()
{
    if (current_state.current_gear == 0)
    {
        current_state.engine_torque = 0;
    }
    else if (current_state.engine_rpm <= params.peak_rpm)
    {
        current_state.engine_torque = current_state.max_torque * (current_state.engine_rpm / params.peak_rpm);
    }
    else
    {
        double drop_factor = 1.0 - 0.4 * (current_state.engine_rpm - params.peak_rpm) / (params.max_rpm - params.peak_rpm);
        current_state.engine_torque = current_state.max_torque * drop_factor;
    }
}

void F1PhysicsEngine::calculateWheelParameters()
{
    if (current_state.current_gear > 0)
    {
        double gear_ratio = params.gear_ratios[current_state.current_gear];
        double gear_factor = gear_ratio * params.final_drive;

        current_state.wheel_based_speed = current_state.wheel_rpm * (2 * M_PI / 60.0) * params.wheel_radius;
        current_state.wheel_torque = current_state.engine_torque * gear_factor;
    }
    else
    {
        current_state.wheel_based_speed = 0.0;
        current_state.wheel_torque = 0.0;
    }
}

// Силы
void F1PhysicsEngine::calculateTractionForce(bool gas_pedal)
{
	if(gas_pedal){
    		current_state.traction_force = current_state.wheel_torque / params.wheel_radius;
	}
	else{
		current_state.traction_force = 0;
	}
}

void F1PhysicsEngine::calculateDragForce()
{
    current_state.drag_force = -0.5 * params.air_density * current_state.velocity.x * abs(current_state.velocity.x) *
                               params.drag_coefficient * params.frontal_area;
}

void F1PhysicsEngine::calculateDownForce()
{
    current_state.down_force = abs(cos(current_state.car_angle)) * params.air_density *
                               abs(current_state.velocity.x) * abs(current_state.velocity.x) *
                               params.downforce_coefficient * params.frontal_area;
}

void F1PhysicsEngine::applyBrakes(double dt)
{
    double brake_diff = current_state.brake_factor * params.brake_rate * dt;
    if (current_state.wheel_rpm - brake_diff >= 0)
    {
        current_state.wheel_rpm -= brake_diff;
    }
}

void F1PhysicsEngine::calculateBrakeForce()
{
    if (current_state.velocity.x <= 0)
    {
        current_state.brake_force = 0;
    }
    else
    {
        current_state.brake_force = -current_state.brake_factor * params.max_brake_force;
    }
}

void F1PhysicsEngine::calculate_Long_Forces(bool gas_pedal, bool brake_pedal)
{
    calculateDragForce();
    calculateBrakeForce();
    calculateDownForce();
    if (gas_pedal) {
    if (current_state.current_gear > 0)
    {
        double gear_ratio = params.gear_ratios[current_state.current_gear];
        double gear_factor = gear_ratio * params.final_drive;
        double max_wheel_rpm = params.max_rpm / gear_factor;
        if (current_state.wheel_rpm >= max_wheel_rpm)
        {
            current_state.traction_force = -current_state.drag_force;
            current_state.wheel_torque = current_state.traction_force * params.wheel_radius;
        }
    }
    } else {
	current_state.traction_force = 0;
    }
    
    if (brake_pedal)
    {
        applyBrakes(0.01);
    }

    current_state.long_force = current_state.traction_force + current_state.drag_force + current_state.brake_force;
}


void F1PhysicsEngine::calculate_Side_Forces(double steering, double dt)
{
    if (current_state.speed < 0.1)
    {
        current_state.side_force = 0;
        return;
    }
    current_state.slip_angle = atan2(current_state.velocity.y, abs(current_state.velocity.x)) - normalizeAngle(current_state.car_angle + current_state.steering_wheel);

    double lateral_force = current_state.slip_angle * params.cornering_stiffness * pow(current_state.speed, 1) * exp(-2 * 0.00005 * (current_state.speed - 180.0) * (current_state.speed - 100.0));

    // Ручной clamp
    current_state.side_force = lateral_force;
}

void F1PhysicsEngine::IsSkeeding(bool gas_pedal)
	
{
    calculateTractionForce(gas_pedal);
   
    double Force = sqrt((current_state.traction_force * current_state.traction_force) +
                        (current_state.side_force * current_state.side_force));
    double max_Force = current_state.current_tire_friction * (params.mass * 9.8 + current_state.down_force);

    if (Force >= max_Force)
    {
        current_state.is_Skeeding = true;

    }
    else
    {
        current_state.is_Skeeding = false;
        
    }

    if (current_state.is_Skeeding)
    {
        max_Force = current_state.current_tire_friction * (params.mass * 9.8 + current_state.down_force);
        double scale_factor = max_Force / Force;
        current_state.traction_force *= scale_factor;
        current_state.side_force *= scale_factor;
    }
}

void F1PhysicsEngine::integrateMotion(double dt)
{
    // --- продольные/поперечные ускорения (оставлено без изменений) ---
    current_state.acceleration.x = current_state.long_force / params.mass;
    current_state.acceleration.y = current_state.side_force / params.mass;

    current_state.velocity.x += current_state.acceleration.x * dt;
    current_state.velocity.y += 0;

    current_state.world_velocity.x = current_state.velocity.x * cos(current_state.car_angle) + current_state.velocity.y * sin(current_state.car_angle);
    current_state.world_velocity.y = current_state.velocity.y * cos(current_state.car_angle) - current_state.velocity.x * sin(current_state.car_angle);

    current_state.position.x += current_state.world_velocity.x * dt;
    current_state.position.y += current_state.world_velocity.y * dt;

    current_state.speed = sqrt(current_state.velocity.x * current_state.velocity.x);

    // УГЛОВОЕ ДВИЖЕНИЕ - правильная логика
    if (current_state.speed > 0.1)
    {
        // Момент от боковой силы
        double torque = current_state.side_force * params.moment_arm;

        // Угловое ускорение
        double angular_acceleration = torque / params.inertia;

        // Обновление угловой скорости
        current_state.angular_velocity += angular_acceleration * dt;

        // Демпфирование
        current_state.angular_velocity -= current_state.angular_velocity * params.rotational_damping * dt;
    }
    else
    {
        // На низких скоростях - сильное демпфирование
        current_state.angular_velocity *= 0.6;
    }

    // Обновление угла автомобиля
    current_state.car_angle += current_state.angular_velocity * dt;
    current_state.car_angle = normalizeAngle(current_state.car_angle);
}

void F1PhysicsEngine::integrateWheelMotion(double dt)
{
    if (current_state.is_Skeeding)
    {
      //  current_state.wheel_rpm += 60 * (current_state.wheel_torque - current_state.traction_force * params.wheel_radius) / (params.wheel_inertia * 2 * M_PI) * dt;
	double acc =  (current_state.wheel_torque - current_state.traction_force * params.wheel_radius) / (params.wheel_inertia) * params.wheel_radius;
        current_state.wheel_speed += acc * dt ;
    }
    else
    {
        current_state.wheel_speed = current_state.speed;
    }
    current_state.wheel_rpm = current_state.wheel_speed * 60 / (params.wheel_radius * 2 * M_PI);

}

void F1PhysicsEngine::CalculateTireParams(double dt)
{
	current_state.current_tire_friction = params.tire_friction[0];
    current_state.slip_ratio = abs(current_state.wheel_speed - current_state.speed) / max(max(current_state.speed, current_state.wheel_speed), 0.1);
    current_state.current_tire_temperature += dt * (params.A_slip_ratio * current_state.slip_ratio + params.A_slip_angle * current_state.slip_angle - params.K_temp * (current_state.current_tire_temperature - params.temp_outside));
    current_state.current_tire_wear += dt * (params.B_slip_ratio * current_state.slip_ratio + params.B_slip_angle * current_state.slip_angle + params.K_wear * max(0.0, (current_state.current_tire_temperature - params.temp_optimal)));
    if (current_state.current_tire_wear > 1.0)
    {
        current_state.current_tire_wear = 1.0;
    }
    if (current_state.current_tire_wear < 0.0)
    {
        current_state.current_tire_wear = 0.0;
    }
    current_state.current_tire_friction = params.tire_friction[0] + (params.tire_friction[1] - params.tire_friction[0]) * current_state.current_tire_wear ;
}

void F1PhysicsEngine::save_to_table(double dt)
{
    ofstream file("F2.csv", ios::app); // ios::app для добавления данных

    static bool first_write = true;
    static double total_time = 0.0;

    if (first_write)
    {
        file << "Time;X;Y;Speed;V_x;V_y;a_x;Car_angle;Steering;Angular_Velocity;Slip_Angle;Skeeding;Tire_Friction;Engine_RPM;Wheel_RPM;Current_gear;Wheel_Torque;Traction_F;Brake_F;Drag_F;Down_F;Long_F;Side_F;Acc"
             << endl;
        first_write = false;
    }

    // Обновляем время (можно передавать как параметр или использовать статическую переменную)
    total_time += dt; // предполагая вызов каждые 10мс

    // Записываем все данные
    file << fixed << setprecision(6)
         << total_time << ";"
         << current_state.position.x << ";"
         << current_state.position.y << ";"
         << current_state.speed << ";"
         << current_state.world_velocity.x << ";"
         << current_state.world_velocity.y << ";"
         << current_state.acceleration.x << ";"
         << current_state.car_angle << ";"
         << current_state.steering_wheel << ";"
         << current_state.angular_velocity << ";"
         << current_state.slip_angle << ";"
         << (current_state.is_Skeeding ? "YES" : "NO") << ";"
         << current_state.current_tire_friction << ";"
         << current_state.engine_rpm << ";"
         << current_state.wheel_rpm << ";"
         << current_state.current_gear << ";"
         << current_state.wheel_torque << ";"
         << current_state.traction_force << ";"
         << current_state.brake_force << ";"
         << current_state.drag_force << ";"
         << current_state.down_force << ";"
         << current_state.long_force << ";"
         << current_state.side_force << ";"
         << current_state.acceleration_rate
         << endl;

    // Для немедленной записи на диск
    file.flush();
}

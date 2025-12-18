#include "F1_Physics.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <vector>
#include <iomanip>
#include <iostream>

using namespace std;

double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}



// Конструктор
F1PhysicsEngine::F1PhysicsEngine(int idx)
{
    reset();
    setTire(idx);
}

void F1PhysicsEngine::setTire(int idx){
    if(idx<0 || idx > params.tires.size()-1){
        cout<<"Wrong tire index"<<endl;
        return;
    }
    current_state.tire_idx = idx;

}

void F1PhysicsEngine::reset()
{
    current_state = CarState();
    current_state.current_gear = 0;
    current_state.current_tire_temperature = 100;
    current_state.current_tire_wear = 0.0;
}

// Основной метод обновления
void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering_wheel)
{
    // 1. Обновление угла руля
    double target_steering = steering_wheel * params.max_steering_wheel;
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
    calculateSideForces();
    calculateLongForces(gas_pedal, brake_pedal);
    isSkidding(gas_pedal);

    integrateMotion(dt);
    integrateWheelMotion(dt);
    calculateTireParams(dt);
    saveToTable(dt);
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

void F1PhysicsEngine::gasUp()
{
    current_state.max_torque += params.dtorque;
    if (current_state.max_torque > params.max_limit_torque)
    {
        current_state.max_torque = params.max_limit_torque;
    }
}

void F1PhysicsEngine::gasDown()
{
    current_state.max_torque -= params.dtorque;
    if (current_state.max_torque < params.min_limit_torque)
    {
        current_state.max_torque = params.min_limit_torque;
    }
}

void F1PhysicsEngine::brakeUp()
{
    current_state.brake_factor += params.brake_factor_k;
    if (current_state.brake_factor > params.brake_max)
    {
        current_state.brake_factor = params.brake_max;
    }
}

void F1PhysicsEngine::brakeDown()
{
    current_state.brake_factor -= params.brake_factor_k;
    if (current_state.brake_factor < params.brake_min)
    {
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
                current_state.engine_rpm = 0;
        }
        else
        {
            current_state.engine_rpm += mean_wheel_load * pow(params.wheel_radius, 2) / params.rotor_inertia * (-current_state.engine_rpm + new_engine_rpm);
            current_state.engine_rpm -= dt * current_state.rotor_resist;

            if (current_state.engine_rpm < 0)
                current_state.engine_rpm = 0;
        }
    }
    else
    {
        current_state.engine_rpm -= current_state.rotor_resist * dt;
        if (gas_pedal)
            current_state.engine_rpm += dt * current_state.acceleration_rate;

        if (current_state.engine_rpm < 0)
            current_state.engine_rpm = 0;
    }

    if (current_state.engine_rpm > params.max_rpm)
        current_state.engine_rpm = params.max_rpm;

    acceleration_rate(current_state.engine_rpm);
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

        current_state.wheel_torque = current_state.engine_torque * gear_factor;
    }
    else
    {
        current_state.wheel_torque = 0.0;
    }
}

// Силы
void F1PhysicsEngine::calculateTractionForce(bool gas_pedal)
{
    if (gas_pedal)
        current_state.traction_force = current_state.wheel_torque / params.wheel_radius;
    else
        current_state.traction_force = 0;
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
        current_state.wheel_rpm -= brake_diff;
}

void F1PhysicsEngine::calculateBrakeForce()
{
    if (current_state.velocity.x <= 0)
        current_state.brake_force = 0;
    else
        current_state.brake_force = -current_state.brake_factor * params.max_brake_force;
}

void F1PhysicsEngine::calculateLongForces(bool gas_pedal, bool brake_pedal)
{
    calculateDragForce();
    calculateBrakeForce();
    calculateDownForce();

    if (gas_pedal)
    {
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
    }
    else
    {
        current_state.traction_force = 0;
    }

    if (brake_pedal)
    {
        applyBrakes(0.01);
    }

    current_state.long_force = current_state.traction_force + current_state.drag_force + current_state.brake_force;
}

void F1PhysicsEngine::calculateSideForces()
{
    if (current_state.speed < 0.1)
    {
        current_state.side_force = 0;
        return;
    }

    current_state.slip_angle = atan2(current_state.velocity.y, abs(current_state.velocity.x)) -
                               normalizeAngle(current_state.car_angle + current_state.steering_wheel);

    double lateral_force = current_state.slip_angle * params.cornering_stiffness  * pow(current_state.speed, 1) *
                           exp(-2 * 0.00005 * (current_state.speed - 180.0) * (current_state.speed - 180.0));

    current_state.side_force = lateral_force;
}

void F1PhysicsEngine::isSkidding(bool gas_pedal)
{
    calculateTractionForce(gas_pedal);

    double Force = sqrt((current_state.traction_force * current_state.traction_force) +
                        (current_state.side_force * current_state.side_force));
    double max_Force = current_state.current_tire_friction * (params.mass * 9.8 + current_state.down_force);

    if (Force >= max_Force)
        current_state.is_skidding = true;
    else
        current_state.is_skidding = false;

    if (current_state.is_skidding)
    {
        max_Force = current_state.current_tire_friction * (params.mass * 9.8 + current_state.down_force);
        double scale_factor = max_Force / Force;
        current_state.traction_force *= scale_factor;
        current_state.side_force *= scale_factor;
    }
}

void F1PhysicsEngine::integrateMotion(double dt)
{
    current_state.acceleration.x = current_state.long_force / params.mass;
    current_state.acceleration.y = current_state.side_force / params.mass;

    current_state.velocity.x += current_state.acceleration.x * dt;
    current_state.velocity.y += 0;

    current_state.world_velocity.x = current_state.velocity.x * cos(current_state.car_angle) + current_state.velocity.y * sin(current_state.car_angle);
    current_state.world_velocity.y = current_state.velocity.y * cos(current_state.car_angle) - current_state.velocity.x * sin(current_state.car_angle);

    current_state.position.x += current_state.world_velocity.x * dt;
    current_state.position.y += current_state.world_velocity.y * dt;

    current_state.speed = sqrt(current_state.velocity.x * current_state.velocity.x);

    if (current_state.speed > 0.1)
    {
        double torque = current_state.side_force * params.moment_arm;
        double angular_acceleration = torque / params.inertia;
        current_state.angular_velocity += angular_acceleration * dt;
        current_state.angular_velocity -= current_state.angular_velocity * params.rotational_damping * dt;
    }
    else
    {
        current_state.angular_velocity *= 0.6;
    }

    current_state.car_angle += current_state.angular_velocity * dt;
    current_state.car_angle = normalizeAngle(current_state.car_angle);
}

void F1PhysicsEngine::integrateWheelMotion(double dt)
{
    if (current_state.is_skidding)
    {
        double acc = (current_state.wheel_torque - current_state.traction_force * params.wheel_radius) /
                     (params.wheel_inertia) * params.wheel_radius;
        current_state.wheel_speed += acc * dt;
    }
    else
    {
        current_state.wheel_speed = current_state.speed;
    }

    current_state.wheel_rpm = current_state.wheel_speed * 60 / (params.wheel_radius * 2 * M_PI);
}

void F1PhysicsEngine::calculateTireParams(double dt)
{
    const TireParams& tire = params.tires[current_state.tire_idx];

    // текущая модель: mu зависит от wear
    current_state.current_tire_friction = tire.tire_friction[0];

    current_state.slip_ratio =
        abs(current_state.wheel_speed - current_state.speed) /
        max(max(current_state.speed, current_state.wheel_speed), 0.1);

    current_state.current_tire_temperature += dt * (
        tire.A_slip_ratio * current_state.slip_ratio +
        tire.A_slip_angle * abs(current_state.slip_angle) -
        tire.K_temp * abs(current_state.current_tire_temperature - tire.temp_outside)
    );
    if(current_state.current_tire_temperature<0){
	   current_state.current_tire_temperature = 0;}


    current_state.current_tire_wear += dt * (
        tire.B_slip_ratio * current_state.slip_ratio +
        tire.B_slip_angle * current_state.slip_angle +
        tire.K_wear * max(0.0, (current_state.current_tire_temperature - tire.temp_optimal))
    );

    if (current_state.current_tire_wear > 1.0) current_state.current_tire_wear = 1.0;
    if (current_state.current_tire_wear < 0.0) current_state.current_tire_wear = 0.0;

    current_state.current_tire_friction =
        tire.tire_friction[0] +
        (tire.tire_friction[1] - tire.tire_friction[0]) * current_state.current_tire_wear;
}

void F1PhysicsEngine::saveToTable(double dt)
{
    std::ofstream file("F2.csv", std::ios::app);

    static bool first_write = true;
    static double total_time = 0.0;

    if (first_write)
    {
        file
            << "Time;"

            // --- Поступательное движение ---
            << "pos_x;pos_y;"
            << "vel_x;vel_y;"
            << "acc_x;acc_y;"
            << "world_vel_x;world_vel_y;"
            << "speed;"
            << "wheel_speed;"
            << "wheel_acceleration;"
            << "linear_slip;"

            // --- Вращательное движение ---
            << "car_angle;"
            << "angular_velocity;"
            << "steering_wheel;"
            << "slip_angle;"

            // --- Трение / шины ---
            << "is_skidding;"
            << "current_tire_friction;"
            << "current_tire_temperature;"
            << "slip_ratio;"
            << "current_tire_wear;"

            // --- Двигатель и трансмиссия ---
            << "max_rpm;"
            << "max_torque;"
            << "acceleration_rate;"
            << "rotor_resist;"
            << "last_engine_rpm;"
            << "engine_rpm;"
            << "engine_torque;"
            << "wheel_rpm;"
            << "last_wheel_rpm;"
            << "wheel_torque;"
            << "current_gear;"
            << "previous_gear;"
            << "wheel_torque_final;"

            // --- Силы ---
            << "traction_force;"
            << "drag_force;"
            << "brake_force;"
            << "down_force;"
            << "long_force;"
            << "side_force;"

            // --- Тормозная система ---
            << "brake_factor"

            << std::endl;

        first_write = false;
    }

    total_time += dt;

    file << std::fixed << std::setprecision(6)
         << total_time << ";"

         // --- Поступательное движение ---
         << current_state.position.x << ";" << current_state.position.y << ";"
         << current_state.velocity.x << ";" << current_state.velocity.y << ";"
         << current_state.acceleration.x << ";" << current_state.acceleration.y << ";"
         << current_state.world_velocity.x << ";" << current_state.world_velocity.y << ";"
         << current_state.speed << ";"
         << current_state.wheel_speed << ";"
         << current_state.wheel_acceleration << ";"
         << current_state.linear_slip << ";"

         // --- Вращательное движение ---
         << current_state.car_angle << ";"
         << current_state.angular_velocity << ";"
         << current_state.steering_wheel << ";"
         << current_state.slip_angle << ";"

         // --- Трение / шины ---
         << (current_state.is_skidding ? 1 : 0) << ";"
         << current_state.current_tire_friction << ";"
         << current_state.current_tire_temperature << ";"
         << current_state.slip_ratio << ";"
         << current_state.current_tire_wear << ";"

         // --- Двигатель и трансмиссия ---
         << current_state.max_rpm << ";"
         << current_state.max_torque << ";"
         << current_state.acceleration_rate << ";"
         << current_state.rotor_resist << ";"
         << current_state.last_engine_rpm << ";"
         << current_state.engine_rpm << ";"
         << current_state.engine_torque << ";"
         << current_state.wheel_rpm << ";"
         << current_state.last_wheel_rpm << ";"
         << current_state.wheel_torque << ";"
         << current_state.current_gear << ";"
         << current_state.previous_gear << ";"
         << current_state.wheel_torque_final << ";"

         // --- Силы ---
         << current_state.traction_force << ";"
         << current_state.drag_force << ";"
         << current_state.brake_force << ";"
         << current_state.down_force << ";"
         << current_state.long_force << ";"
         << current_state.side_force << ";"

         // --- Тормозная система ---
         << current_state.brake_factor

         << std::endl;

    file.flush();
}

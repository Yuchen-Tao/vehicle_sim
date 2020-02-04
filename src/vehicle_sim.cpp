#include "vehicle_sim.h"

typedef std::chrono::high_resolution_clock ChronoTime;

Vehicle::Vehicle() {
    // parameter initialization
    delta_f = 0;
    beta = 0;
    vel = 0;
    x = 0;
    y = 0;
    psi = 0;

    acc = 0;
    delta_f = 0;

    time_counter = 0;
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("SDL2 Keyboard/Mouse events", SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED, 200, 200, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
}

void Vehicle::update_state() {
    beta = atan((l_r / (l_f + l_r) * tan(delta_f)));
    x = x + vel * cos(psi + beta) * dt;
    y = y + vel * sin(psi + beta) * dt;
    psi = psi + (vel / l_r) * sin(beta) * dt;
    double force = b * abs(vel);
    vel = vel + (acc - copysign(force / mass, vel)) * dt;
}

void Vehicle::create_message(ros::NodeHandle& n, ros::Publisher& msg_pub_) {

    // Publish Object_State message
    double cy = cos(psi * 0.5);
    double sy = sin(psi * 0.5);
    double cp = cos(0 * 0.5);
    double sp = sin(0 * 0.5);
    double cr = cos(0 * 0.5);
    double sr = sin(0 * 0.5);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg_class.classification = 4; // CAR
    msg_class.probability = 1;
    msg.classification.classes_with_probabilities.emplace_back(msg_class);
    msg.motion_state.header.frame_id = "map";
    msg.motion_state.pose.pose.position.x = x;
    msg.motion_state.pose.pose.position.y = y;
    msg.motion_state.pose.pose.position.z = 0;
    // Euler Angles to Quaternion Conversion
    msg.motion_state.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
    msg.motion_state.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    msg.motion_state.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    msg.motion_state.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;

    msg.motion_state.twist.twist.linear.x = vel * cos(psi + beta);
    msg.motion_state.twist.twist.linear.y = vel * sin(psi + beta);
    msg.motion_state.twist.twist.linear.z = 0;

    msg_array.header.stamp = ros::Time::now();
    msg_array.header.frame_id = "map";
    msg_array.objects.emplace_back(msg);
    msg_pub_.publish(msg_array);
    // msg_pub_.publish(msg);
}

void Vehicle::keyboard_control(ros::NodeHandle& n, ros::Publisher& msg_pub_) {
    SDL_Event event;
    int quit = 0;

    std::chrono::duration<double> loop_dt{dt};

    // Get loop start time
    auto end_time = ChronoTime::now() + std::chrono::duration<double>(0);
    std::chrono::duration<double> cycle_time;
    std::chrono::duration<double> sleep_time;

    double angle_to_rad = M_PI / 180;

    while (!quit) {
        // calculate desired loop end time based on desired sample rate
        end_time += std::chrono::duration<double>(loop_dt);

        update_state();
        create_message(n, msg_pub_);
        msg_array.objects.clear();
        msg.classification.classes_with_probabilities.clear();

        // The keyboard controls delta_f and acceleration by left/right/up/down,
        // press Esc to quit the simulation.

        const Uint8* state = SDL_GetKeyboardState(NULL);
        if (state[SDL_SCANCODE_UP] && acc < 10)
            acc += 0.2;
        else if (state[SDL_SCANCODE_DOWN] && acc > -10)
            acc -= 0.2;
        else
            acc = 0;
        if (state[SDL_SCANCODE_LEFT] && delta_f < 70 * angle_to_rad)
            // delta_f += 1.0 * angle_to_rad;
            delta_f = 45 * angle_to_rad;
        else if (state[SDL_SCANCODE_RIGHT] && delta_f > -70 * angle_to_rad)
            // delta_f -= 1.0 * angle_to_rad;
            delta_f = -45 * angle_to_rad;
        else
            delta_f = 0;

        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            // Look for a keypress
            case SDL_KEYDOWN:
                // Check the SDLKey values and move change the coords
                switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    ROS_INFO("%s", "quit simulation");
                    quit = 1;
                    break;
                default:
                    break;
                }
                break;
            case SDL_QUIT:
                ROS_INFO("%s", "quit simulation");
                quit = 1;
                break;
            default:
                break;
            }
        }

        // Sleep until prediction sample rate is reached
        sleep_time = end_time - ChronoTime::now();
        cycle_time = loop_dt - sleep_time;
        if (cycle_time < loop_dt) {
            std::this_thread::sleep_until(end_time);
        } else {
            std::cerr << "Warning! Current cycle time is " << cycle_time.count() << " > "
                      << loop_dt.count() << std::endl;
            end_time = ChronoTime::now();
        }
    }
}

# Vision algorithm

### The code

```c++

inline TargetData calculate_target_data(const uav_msgs::msg::Vision::SharedPtr msg)
{
    // Vision Resolution 640 * 640
    float x = (float(msg->red_x + msg->blue_x - 640)) / 2.0f;
    float y = (float(msg->red_y + msg->blue_y - 640)) / 2.0f;
    float width = (float(msg->red_width + msg->blue_width)) / 2.0f;
    float height = (float(msg->red_height + msg->blue_height)) / 2.0f;
    float size = std::max(width, height);
    float delta_x = msg->blue_x - msg->red_x;
    float delta_y = msg->blue_y - msg->red_y;
    
    // Real world size of targets in meters
    float target_size = 0.187f;

    // Create forward vector (vec1) and target vector (vec2)
    // Camera is positioned such that forward is to the right
    struct Vector2 { float x, y; };
    Vector2 vec1 = { 1.0f, 0.0f };
    Vector2 vec2 = { delta_x, -delta_y };

    // Calculate angle difference of forward vector
    // and target vector (red target to blue target)
    float dot_product = vec1.x * vec2.x + vec1.y * vec2.y;
    float mag1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
    float mag2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    float cos_theta = dot_product / (mag1 * mag2);
    float angle_radians = std::acos(cos_theta);

    // Cross product to find if angle is positive or negative
    float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
    if (cross_product < 0)
        angle_radians = -angle_radians;

    //float angle_degrees = angle_rad * (180.0f / M_PI);

    // Found this value through trial and error. Probably not 100 % right
    // Increase = bigger / decrease = smaller TargetData values.
    constexpr float focal_length = 850.0f;

    TargetData target = {};
    target.depth = target_size * focal_length / size;
    target.side_axis = -y * target.depth / focal_length;
    target.travel_axis = x * target.depth / focal_length;
    target.angle = angle_radians; // angle_degrees
    return target;
}
```

This is the way we detect the targets.

When developing an algorithm for autonomous flight, this is what that should be based on.

The return value is a struct that contains:

### depth:
How far away the targets are in meters

### side_axis:
How far to the side of the camera, the average position of the targets are.
If the target is to the right of the drone, this value is positive.

### travel_axis:
How far in front of the camera, the average position of the targets are.
If the target is in front of the drone, this value is positive.

### angle
The angular difference of the target vector (red target towards blue target) and the forward direction of the drone.
If the drone is rotated clockwise, this value is positive. Right now we use radians.

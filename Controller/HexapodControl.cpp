//
// Command-line tool to publish velocity, heading and height commands
// to the HexapodController via Gazebo transport topics.
//
// Usage:
//   HexapodControl --velocity <mm/s> --heading <rad> [--height <mm>]
//

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

#include <gz/transport.hh>
#include <gz/msgs.hh>

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --velocity <value>   Forward velocity (mm/s)\n");
    fprintf(stderr, "  --heading  <value>   Heading angle (radians)\n");
    fprintf(stderr, "  --height   <value>   Body height (mm, default: unchanged)\n");
}

int main(int argc, char *argv[]) {
    bool set_velocity = false;
    bool set_heading  = false;
    bool set_height   = false;
    double velocity   = 0.0;
    double heading    = 0.0;
    double height     = 0.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--velocity") == 0 && i + 1 < argc) {
            velocity = atof(argv[++i]);
            set_velocity = true;
        } else if (strcmp(argv[i], "--heading") == 0 && i + 1 < argc) {
            heading = atof(argv[++i]);
            set_heading = true;
        } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
            height = atof(argv[++i]);
            set_height = true;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (!set_velocity && !set_heading && !set_height) {
        fprintf(stderr, "Error: at least one of --velocity, --heading, or --height is required\n");
        usage(argv[0]);
        return 1;
    }

    gz::transport::Node node;

    gz::transport::Node::Publisher velocity_pub;
    gz::transport::Node::Publisher heading_pub;
    gz::transport::Node::Publisher height_pub;

    if (set_velocity) {
        velocity_pub = node.Advertise<gz::msgs::Double>(
            "/world/hexspider_world/model/hexspider/velocity");
        if (!velocity_pub) {
            fprintf(stderr, "Failed to advertise velocity topic\n");
            return 1;
        }
    }

    if (set_heading) {
        heading_pub = node.Advertise<gz::msgs::Double>(
            "/world/hexspider_world/model/hexspider/heading");
        if (!heading_pub) {
            fprintf(stderr, "Failed to advertise heading topic\n");
            return 1;
        }
    }

    if (set_height) {
        height_pub = node.Advertise<gz::msgs::Double>(
            "/world/hexspider_world/model/hexspider/height");
        if (!height_pub) {
            fprintf(stderr, "Failed to advertise height topic\n");
            return 1;
        }
    }

    // Wait for the controller's subscribers to discover this publisher via the
    // gz-transport discovery protocol. Discovery over localhost typically takes
    // 100-300ms; 500ms gives a comfortable margin.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (set_velocity) {
        gz::msgs::Double msg;
        msg.set_data(velocity);
        velocity_pub.Publish(msg);
        printf("velocity -> %.4f\n", velocity);
    }

    if (set_heading) {
        gz::msgs::Double msg;
        msg.set_data(heading);
        heading_pub.Publish(msg);
        printf("heading  -> %.4f\n", heading);
    }

    if (set_height) {
        gz::msgs::Double msg;
        msg.set_data(height);
        height_pub.Publish(msg);
        printf("height   -> %.4f\n", height);
    }

    // Give gz-transport time to deliver the messages before the node is destroyed
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return 0;
}

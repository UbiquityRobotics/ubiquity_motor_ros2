//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//
#include <iostream>
#include <string>
#include <sstream>
#include "zenoh.hxx"
using namespace zenoh;


// Vector3 structure
struct Vector3 {
    double x, y, z;

    // Serialize this Vector3 to a string (for example, JSON format)
    std::string serialize() const {
        std::ostringstream ss;
        ss << "{\"x\":" << x << ",\"y\":" << y << ",\"z\":" << z << "}";
        return ss.str();
    }
};

// Twist structure
struct Twist {
    Vector3 linear;
    Vector3 angular;

    // Serialize this Twist to a string (for example, JSON format)
    std::string serialize() const {
        std::ostringstream ss;
        ss << "{\"linear\":" << linear.serialize()
           << ",\"angular\":" << angular.serialize() << "}";
        return ss.str();
    }
};

int main(int, char **) {
    Config config = Config::create_default();
    auto session = Session::open(std::move(config));

    auto pub = session.declare_publisher(KeyExpr("turtle1/cmd_vel")); 

    // Create a Twist instance
    Twist t = {Vector3{2.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}};
    std::cout << "Size of Twist: " << sizeof(t) << " bytes" << std::endl;


    // Serialize the Twist instance
    std::string serialized_twist = t.serialize();

    // std::cout << "Size of seralized Twist: " << sizeof(serialized_twist) << " bytes" << std::endl;

    Publisher::PutOptions options;
    options.encoding = Encoding("text/plain");

    pub.put(Bytes::serialize(serialized_twist), std::move(options));
    // pub.put(Bytes::serialize(serialized_twist), std::move(options));
    // pub.put(Bytes::serialize(serialized_twist), std::move(options));
    // pub.put("rt/turtle1/cmd_vel", serialized_twist);
    // pub.put("rt/turtle1/cmd_vel", serialized_twist);
}


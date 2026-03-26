#include "cv1/robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char **argv) {
rclcpp::init(argc, argv);

Robot robot;
char command;

while (true){
std::cout << "Pre pohyb w/a/s/d, q pre odchod \n"; //console output << operator pre poslanie do vystupu
std::cin >> command;

if (command == 'q'){
break;
}
if (command == 'w'||command == 'W'){
robot.move(0, 1);
} else if (command == 'a'||command == 'A'){ 
robot.move(-1, 0);
} else if (command == 's'||command == 'S'){ 
robot.move(0, -1);
} else if (command == 'd'||command == 'D'){ 
robot.move(1, 0);
}

std::cout << "Aktualna pozicia: ("<<robot.getX()<<", "<<robot.getY()<<")"<<std::endl;
}
rclcpp::shutdown();
return 0;
}

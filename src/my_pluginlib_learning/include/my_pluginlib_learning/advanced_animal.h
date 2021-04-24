#ifndef _ADVANCED_ANIMAL_CLASS_
#define _ADVANCED_ANIMAL_CLASS_

#include <my_pluginlib_learning/base_animal.h>
#include <ros/ros.h>
namespace walkingAnimals{
    class cat : public baseAnimal::canRun{
        public:
        cat(){
            ROS_INFO("cat created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("cat name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("cat make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("cat move %d step.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
    class dog : public baseAnimal::canRun{
        public:
        dog(){
            ROS_INFO("dog created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("dog name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("dog make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("dog move %d step.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
};
namespace flyingAnimals{
    class pigeon : public baseAnimal::canFly{
        public:
        pigeon(){
            ROS_INFO("pigeon created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("pigeon name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("pigeon make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("pigeon fly %d meter.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
    class eagle : public baseAnimal::canFly{
        public:
        eagle(){
            ROS_INFO("eagle created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("eagle name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("eagle make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("eagle fly %d meter.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
};
namespace swimingAnimals{
    class fish : public baseAnimal::canSwim{
        public:
        fish(){
            ROS_INFO("fish created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("fish name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("fish can't make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("fish swim %d meter.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
    class duck : public baseAnimal::canSwim{
        public:
        duck(){
            ROS_INFO("duck created");
            totalMove_ = 0;
            return;
        };
        void init(std::string name){
            name_ = name;
            ROS_INFO("duck name: %s", name.c_str());
            return;
        };
        void makeSound(){
            ROS_INFO("duck  make sound");
            return;
        };
        void move(int len = 1){
            ROS_INFO("duck swim %d meter.", len);
            totalMove_ += len;
        };
        protected:
        int totalMove_;
    };
};


#endif
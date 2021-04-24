#ifndef _BASE_ANIMAL_CLASS_
#define _BASE_ANIMAL_CLASS_

#include <string>

// 构造函数不能有参数
// 类继承应为共有继承

namespace baseAnimal{
    class canRun{
        public:
        virtual void init(std::string name) = 0;
        virtual void makeSound() = 0;
        virtual void move(int len) = 0;
        virtual ~canRun(){};
        std::string getName(){
            return name_;
        }
        protected:
        std::string name_;
        canRun(){};
    };
    class canFly{
        public:
        virtual void init(std::string name) = 0;
        virtual void makeSound() = 0;
        virtual void move(int len) = 0;
        virtual ~canFly(){};
        std::string getName(){
            return name_;
        }
        protected:
        std::string name_;
        canFly(){};
    };
    class canSwim{
        public:
        virtual void init(std::string name) = 0;
        virtual void makeSound() = 0;
        virtual void move(int len) = 0;
        virtual ~canSwim(){};
        std::string getName(){
            return name_;
        }
        protected:
        std::string name_;
        canSwim(){};
    };
};

#endif
#include <pluginlib/class_loader.h>
#include <my_pluginlib_learning/base_animal.h>
#include <boost/shared_ptr.hpp>

int main(int argc, char* argv[]){
    // 利用描述文件中描述的基类, 生成生成器
    pluginlib::ClassLoader<baseAnimal::canFly> flyAnimalLoader("my_pluginlib_learning", "baseAnimal::canFly");
    pluginlib::ClassLoader<baseAnimal::canRun> runAnimalLoader("my_pluginlib_learning", "baseAnimal::canRun");
    pluginlib::ClassLoader<baseAnimal::canSwim> swimAnimalLoader("my_pluginlib_learning", "baseAnimal::canSwim");
    try{
        // 根据描述的派生类生成指针
        boost::shared_ptr<baseAnimal::canRun> cat = runAnimalLoader.createInstance("walkingAnimals::cat");
        cat->init("tom");
        cat->makeSound();
        cat->move(10);
    }
    catch(pluginlib::PluginlibException& ex){
        ROS_ERROR("plugin load fail, err: %s.", ex.what());
    }
    return 0;
}
#include <ros/ros.h>
#include <ros/package.h>
#include <pluginlib/class_loader.h>
#include <my_pluginlib_learning/base_animal.h>
#include <boost/shared_ptr.hpp>

template <class T> class growInstance{
    public:
    boost::shared_ptr<T> instancePtr_;
    bool avail_ = false;// 判断是否称该加载实例
    growInstance(std::string pkgName, std::string baseName, std::string instanceName){
        // std::string pkgName_;
        // ros::package::getPath(pkgName_);
        // ROS_INFO("%s", pkgName_.c_str());
        baseClassLoader_ = new pluginlib::ClassLoader<T>(pkgName.c_str(), baseName.c_str());
        try{
            instancePtr_ = baseClassLoader_->createInstance(instanceName);
        }
        catch (pluginlib::PluginlibException& ex){
            ROS_ERROR("plugin load fail, err: %s.", ex.what());
            return;
        }
        avail_ = true;
        return;
    }
    ~growInstance(){
        // 干掉loader前要先干掉实例，否则报错
        // SEVERE WARNING!!! 
        // Attempting to unload library while objects created by 
        // this loader exist in the heap! You should delete your objects 
        // before attempting to unload the library or destroying the ClassLoader. 
        // The library will NOT be unloaded.
        ROS_INFO("%s: deleted.", instancePtr_->getName().c_str());
        instancePtr_.reset();
        delete baseClassLoader_;
        return;
    }
    private:
    ros::NodeHandle handler_;
    // 用指针是为了避免使用初始化列表
    pluginlib::ClassLoader<T>* baseClassLoader_;
};

int main(int argc, char* argv[]){
    // 利用描述文件中描述的基类, 生成生成器
    ros::init(argc, argv, "plugin_caller_node");
    ros::NodeHandle handler;
    std::string runClassType;
    std::string flyClassType;
    std::string swimClassType;
    if (handler.getParam("can_run_animal", runClassType)){
        growInstance<baseAnimal::canRun> catInstance("my_pluginlib_learning", "baseAnimal::canRun", runClassType);
        if(catInstance.avail_){
            catInstance.instancePtr_->init("pluto");
            catInstance.instancePtr_->makeSound();
            catInstance.instancePtr_->move(10);
        }
    }
    if (handler.getParam("can_fly_animal", flyClassType)){
        growInstance<baseAnimal::canFly> flyInstance("my_pluginlib_learning", "baseAnimal::canFly", flyClassType);
        if(flyInstance.avail_){
            flyInstance.instancePtr_->init("lulu");
            flyInstance.instancePtr_->makeSound();
            flyInstance.instancePtr_->move(10);
        }
    }
    if (handler.getParam("can_swim_animal", swimClassType)){
        growInstance<baseAnimal::canSwim> swimInstance("my_pluginlib_learning", "baseAnimal::canSwim", swimClassType);
        if(swimInstance.avail_){
            swimInstance.instancePtr_->init("DD");
            swimInstance.instancePtr_->makeSound();
            swimInstance.instancePtr_->move(10);
        }
    }
    return 0;
}
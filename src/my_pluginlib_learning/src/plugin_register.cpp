#include <pluginlib/class_list_macros.h>
#include <my_pluginlib_learning/base_animal.h>
#include <my_pluginlib_learning/advanced_animal.h>
// https://blog.csdn.net/sunbibei/article/details/52958724
// 没有分号
// 1.添加头文件, 2.本文件, 3.cmakelist后就会生成.so文件
// 基类定义的接口要和派生类的相同,否则无法调用
// 下面的第一个参数是派生类, 第二个参数是基类
// 分别查看cmakelist, animal_plugin.xml, package.xml, plugin_caller.cpp查看调用流程
PLUGINLIB_EXPORT_CLASS(walkingAnimals::cat, baseAnimal::canRun)
PLUGINLIB_EXPORT_CLASS(walkingAnimals::dog, baseAnimal::canRun)
PLUGINLIB_EXPORT_CLASS(swimingAnimals::fish, baseAnimal::canSwim)
PLUGINLIB_EXPORT_CLASS(swimingAnimals::duck, baseAnimal::canSwim)
PLUGINLIB_EXPORT_CLASS(flyingAnimals::pigeon, baseAnimal::canFly)
PLUGINLIB_EXPORT_CLASS(flyingAnimals::eagle, baseAnimal::canFly)

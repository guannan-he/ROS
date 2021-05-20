# ROS packages
1. topic 发送订阅:  
    my_talker_node 通过三个话题向 my_listener_node 发送两种消息  
    运行命令：  
    ```
    roslaunch topic_pub_sub topic_pub_sub.launch
    ```  
    [topic_pub_sub](https://github.com/guannan-he/ROS/tree/main/src/topic_pub_sub)  
    ![image](images/topic_pub_sub/nodes.png)  
2. service 客户端、服务器:  
    my_client_node 向 my_server_node 发送服务请求，my_server_node 视数据有效性决定是否拒绝服务  
    运行命令：  
    ```
    roslaunch service_req_rep service_req_rep.launch
    ```  
    [service_req_rep](https://github.com/guannan-he/ROS/tree/main/src/service_req_rep)  
    ![image](images/service_req_rep/nodes.png)  
3. param 服务器:  
    dynamic_configure_node 收到参数变化请求后调用 myParamDynamicSetCallServer 提供的服务  
    运行命令：  
    ```
    roslaunch param_dynamic_set param_dynamic_set.launch
    ```  
    [param_dynamic_set](https://github.com/guannan-he/ROS/tree/main/src/param_dynamic_set)  
    ![image](images/param_dynamic_set/nodes.png)  
4. 小乌龟TF:  
    通过键盘控制乌龟1位置，乌龟2订阅TF树上`乌龟1上参考点`相对于`乌龟2`的变换，乌龟2跟踪该变换并设法使变换归零。跟踪目标可以通过`start_demo.launch`修改  
    运行命令：  
    ```
    roslaunch learning_tf start_demo.launch
    ```  
    [learning_tf](https://github.com/guannan-he/ROS/tree/main/src/learning_tf)  
    ![image](images/learning_tf/nodes.png)  
5. 动作服务器:  
    randNumGen 生成随机数发布到 randomNumber 话题，avgActionClient 设定目标并接受 avgActionServer 提供的反馈  
    运行命令：  
    ```
    roslaunch action_server_client server_and_client_avg.launch
    ```  
    [topic_pub_sub](https://github.com/guannan-he/ROS/tree/main/src/action_server_client)  
    ![image](images/action_server_client/nodes.png)  
6. pluginlib:  
    pluginlib 利用面向对象编程的继承概念，在`基类`中定义方法，在`继承类`中实现  
    推荐使用`公有继承`  
    运行命令：  
    ```
    roslaunch my_pluginlib_learning plugin_param_demo.launch
    ```  
    ROS中插件注册插件流程：  
    1) 编写`基类`和`继承类`的头文件，并使`继承类`继承`基类`的接口  
    2) 新建`源文件`并分别添加`基类`和`继承类`的头文件、`pluginlib/class_list_macros.h`头文件
    3) 在`源文件`中使用 `PLUGINLIB_EXPORT_CLASS(`继承类`, `基类`)` 指定`基类`和`继承类`的关系  
    4) 在 CMakeLists.txt 中添加如下代码以生成名为`lib插件名称.so`的动态链接库  
        ```
        add_library(插件名称 源文件)
        target_link_libraries(插件名称 ${catkin_LIBRARIES})
        ```
    5) 新建`插件描述.xml`对插件继承关系进行描述  
        ```
        <library path = "lib/lib插件名称">
            <class name = "任意起一个名字" type = "继承类" base_class_type = "基类">
                <description>继承类用途描述</description>
            </class>
        </library>
        ```
    6) 在`package.xml`中`<export>`标签下添加如下代码，将该插件注册到某一插件库中  
        ```
        <插件库名称 plugin = "${prefix}/插件描述.xml"/>
        ```  
    7) 编译后在终端中输入如下代码检查插件时否正确注册  
        ```
        rospack plugins --attrib=plugin 插件库名称
        ```  
    8) 在源文件中使用`基类`生成`继承类`实例，查看[plugin_caller.cpp](https://github.com/guannan-he/ROS/blob/main/src/my_pluginlib_learning/src/plugin_caller.cpp)查看具体使用方式，生成`继承类`既可以使用`插件描述.xml`中定义的名字，也可以使用`继承类`名称  
7. nodelet:  
8. debug：
9. lasis_vehicle: 正在实现  
10. my_global_planner_plugin  
 
    
# references  
[pluginlib_tutorials](https://github.com/huchunxu/ros_blog_sources/tree/master/pluginlib_tutorials)  
[教程](https://haoqchen.site/2019/08/15/debug-ros-with-vscode/)  
[参考](https://github.com/xmy0916/racecar)  
[carrot planner](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)  
[aStar planner& dijkstra planner](https://zhuanlan.zhihu.com/p/113662488)  
[NavFnROS中势能计算](https://github.com/locusrobotics/robot_navigation/tree/master/dlux_global_planner#the-kernel) 


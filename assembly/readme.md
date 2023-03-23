# 流程

[大爹文章](https://blog.csdn.net/lc1852109/article/details/126450496?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522167954793316800222818444%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=167954793316800222818444&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-126450496-null-null.142^v76^control_1,201^v4^add_ask,239^v2^insert_chatgpt&utm_term=gazebo%E6%B7%BB%E5%8A%A0%E6%8E%A7%E5%88%B6%E6%8F%92%E4%BB%B6&spm=1018.2226.3001.4187)

## １.能加载urdf文件到参数服务器

### 见display.launch 以及assembly.urdf 不要再动

### rviz加载以后可能注释

##  2.能在gazebo加载urdf文件的模型

### 见gazebo.launch 的前三句标签　第一句加载gazebo app以后可能注释

## ３.将urdf文件转成sdf文件，配置models文件夹(.sdf	.config	meshes文件夹)，添加路径并能在gazebo的add path找到模型

### gz sdf -p my_model.urdf > my_model.sdf

#### a)models下的robot文件夹添加到.gazebo/models里面，在gazebo默认路径可以加载

#### b)gazebo里面insert add path进入models里面，即可加载出模型

### 注意：urdf文件下的路径格式为:(其中assembly2为功能包名称)

```
<mesh filename="package://assembly2/models/robot/meshes/Empty_right_Link.STL" />
```

### 到sdf文件下的路径格式为:(其中model://直接跟sdf文件上级文件夹robot即可，也方便移植到.gazebo/models里面)

```
<uri>model://robot/meshes/base_link.STL</uri>
```

## 4.将sdf文件添加配置变为.world文件并用launch文件加载模型到gazebo

### a)建立world文件夹并添加.world文件，里面内容参考模板即可，model://跟sdf文件上级文件夹robot

```
gazebo assembly.world 
```

#### 此时在gazebo里面能看到sdf模型，其中gazebo里面的模型名字是world里面设置的model name

#### (由此可见，最好将robot文件夹复制到.gazebo/models比较省心)

### b)launch文件集成:

#### 	见gazebo.launch的第二个include ,此时把前面三个标签注释掉，那个是前文提到的加载urdf模型

# 试一试小插件能不能行

## 1.用gazebo的gz命令加载插件

### 	cmakelists配置参考assembly/Cmakelists.txt

### 	.zshrc添加路径(不知道有没有用)

### 	插件注册时是model类型，所以在world文件的model标签里面里面添加插件，见assembly.world

```
 gzserver -u assembly.world
```

### 另外终端输入gzclient	即可加载出带有插件影响的模型

## 2.launch文件配置插件

### 直接集成在launch文件，注意一个细节

```
      <plugin name="Empty_front_Joint" filename="libmy_joint_controller.so">
        <jointname>assembly::Empty_front_Joint</jointname>
        <p_gain_pos>1</p_gain_pos>
        <i_gain_pos>0</i_gain_pos>
        <d_gain_pos>0</d_gain_pos>
        <p_gain_vel>1</p_gain_vel>
        <i_gain_vel>0</i_gain_vel>
        <d_gain_vel>0</d_gain_vel>
      </plugin>
```

### name随便取

### jointname的名字一定是assembly.sdf里面的关节名字

### 同时前面要加域名,也就是.sdf里面的robot name 即assembly,否则加载不到关节

### 此时看到话题名称:

#### /Dipan/assembly/Empty_front_Joint/pos_cmd

#### /Dipan/assembly/Empty_front_Joint/pose

#### /Dipan/assembly/Empty_front_Joint/vel_cmd




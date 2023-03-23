# 流程

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

## 5.试一试小插件能不能行
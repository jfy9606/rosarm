# 单个目标检测结果的消息定义

# 目标类别信息
int32 class_id       # 类别ID
string class_name    # 类别名称
float32 confidence   # 置信度

# 边界框信息
int32 x_min          # 左上角x坐标
int32 y_min          # 左上角y坐标
int32 x_max          # 右下角x坐标
int32 y_max          # 右下角y坐标

# 中心点和尺寸
float32 center_x     # 中心点x坐标
float32 center_y     # 中心点y坐标
float32 width        # 宽度
float32 height       # 高度 
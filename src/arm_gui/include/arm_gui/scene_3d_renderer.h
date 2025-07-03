#ifndef SCENE_3D_RENDERER_H
#define SCENE_3D_RENDERER_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector3D>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>
#include <vector>
#include <utility>

// Scene3DRenderer类定义
class Scene3DRenderer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    Scene3DRenderer(QWidget* parent = nullptr);
    ~Scene3DRenderer();

    void updateObjects(const std::vector<std::pair<QVector3D, QColor>>& objects);
    void setSelectedObject(int index);
    int getSelectedObject() const;
    void setRobotPose(const std::vector<double>& joint_values);
    void setEndEffectorPosition(const QVector3D& position);
    QVector3D getEndEffectorPosition() const;

signals:
    void objectSelected(int index);
    void endEffectorPositionChanged(QVector3D position);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    // 渲染函数
    void renderCoordinateAxes();
    void renderRobot();
    void renderObjects();
    
    // 射线拾取相关
    QVector3D screenToRay(int x, int y);
    bool rayIntersectsPlane(const QVector3D& ray_origin, const QVector3D& ray_direction,
                            const QVector3D& plane_point, const QVector3D& plane_normal,
                            float& t_hit);
    bool rayIntersectsSphere(const QVector3D& ray_origin, const QVector3D& ray_direction,
                             const QVector3D& sphere_center, float sphere_radius);
    
    // 成员变量
    std::vector<std::pair<QVector3D, QColor>> objects_;
    int selected_object_;
    
    // 相机
    QVector3D camera_position_;
    QVector3D camera_target_;
    QVector3D camera_up_;
    
    // 相机参数
    float yaw_;
    float pitch_;
    float zoom_;
    
    // 鼠标交互
    QPoint last_mouse_pos_;
    
    // 机械臂状态
    std::vector<double> robot_joints_;
    
    // 矩阵
    QMatrix4x4 view_matrix_;
    QMatrix4x4 projection_matrix_;
    
    // 末端执行器相关
    QVector3D end_effector_position_;
    bool dragging_end_effector_;
    QVector3D drag_offset_;
};

#endif // SCENE_3D_RENDERER_H 
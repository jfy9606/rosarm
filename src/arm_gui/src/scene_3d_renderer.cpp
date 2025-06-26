#include "arm_gui/arm_control_gui.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <QOpenGLFunctions>
#include <cmath>

// Scene3DRenderer 实现
Scene3DRenderer::Scene3DRenderer(QWidget* parent)
    : QOpenGLWidget(parent)
    , selected_object_(-1)
    , camera_position_(0.0f, 0.0f, 5.0f)
    , camera_target_(0.0f, 0.0f, 0.0f)
    , camera_up_(0.0f, 1.0f, 0.0f)
    , yaw_(-90.0f)
    , pitch_(0.0f)
    , zoom_(5.0f)
{
    // 设置焦点策略，允许通过Tab键获取焦点
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

Scene3DRenderer::~Scene3DRenderer()
{
    // 清理OpenGL资源
}

void Scene3DRenderer::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    // 设置光源
    GLfloat light_position[] = { 1.0f, 1.0f, 1.0f, 0.0f };
    GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
}

void Scene3DRenderer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<float>(w) / h, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void Scene3DRenderer::paintGL()
{
    // 清除缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // 更新摄像机位置
    float camX = camera_target_.x() + zoom_ * cos(yaw_ * M_PI / 180.0f) * cos(pitch_ * M_PI / 180.0f);
    float camY = camera_target_.y() + zoom_ * sin(pitch_ * M_PI / 180.0f);
    float camZ = camera_target_.z() + zoom_ * sin(yaw_ * M_PI / 180.0f) * cos(pitch_ * M_PI / 180.0f);
    
    camera_position_ = QVector3D(camX, camY, camZ);
    
    // 设置视图
    gluLookAt(
        camera_position_.x(), camera_position_.y(), camera_position_.z(),
        camera_target_.x(), camera_target_.y(), camera_target_.z(),
        camera_up_.x(), camera_up_.y(), camera_up_.z()
    );
    
    // 绘制坐标轴
    renderCoordinateAxes();
    
    // 绘制机械臂
    renderRobot();
    
    // 绘制物体
    renderObjects();
}

void Scene3DRenderer::mousePressEvent(QMouseEvent* event)
{
    last_mouse_pos_ = event->pos();
    
    if (event->button() == Qt::LeftButton) {
        // 检测射线与物体相交
        QVector3D ray = screenToRay(event->x(), event->y());
        
        // 检查射线与每个物体的相交
        bool found = false;
        for (size_t i = 0; i < objects_.size(); ++i) {
            if (rayIntersectsSphere(camera_position_, ray, objects_[i].first, 0.1f)) {
                selected_object_ = i;
                found = true;
                emit objectSelected(i);
                update();
                break;
            }
        }
        
        if (!found) {
            selected_object_ = -1;
            emit objectSelected(-1);
            update();
        }
    }
}

void Scene3DRenderer::mouseMoveEvent(QMouseEvent* event)
{
    if (event->buttons() & Qt::RightButton) {
        // 旋转视角
        int dx = event->x() - last_mouse_pos_.x();
        int dy = event->y() - last_mouse_pos_.y();
        
        yaw_ += dx * 0.5f;
        pitch_ += dy * 0.5f;
        
        // 限制pitch角度
        if (pitch_ > 89.0f)
            pitch_ = 89.0f;
        if (pitch_ < -89.0f)
            pitch_ = -89.0f;
        
        update();
    } else if (event->buttons() & Qt::MiddleButton) {
        // 平移视角
        int dx = event->x() - last_mouse_pos_.x();
        int dy = event->y() - last_mouse_pos_.y();
        
        float speed = 0.01f;
        QVector3D right = QVector3D::crossProduct(
            QVector3D(0, 1, 0), camera_position_ - camera_target_).normalized();
        QVector3D up = QVector3D::crossProduct(
            camera_position_ - camera_target_, right).normalized();
        
        camera_target_ += right * dx * speed * zoom_;
        camera_target_ += up * dy * speed * zoom_;
        
        update();
    }
    
    last_mouse_pos_ = event->pos();
}

void Scene3DRenderer::wheelEvent(QWheelEvent* event)
{
    // 缩放视角
    float delta = event->angleDelta().y() / 120.0f;
    zoom_ -= delta * 0.5f;
    
    // 限制缩放范围
    if (zoom_ < 1.0f)
        zoom_ = 1.0f;
    if (zoom_ > 20.0f)
        zoom_ = 20.0f;
    
    update();
}

void Scene3DRenderer::updateObjects(const std::vector<std::pair<QVector3D, QColor>>& objects)
{
    objects_ = objects;
    update();
}

void Scene3DRenderer::setSelectedObject(int index)
{
    if (index >= -1 && index < static_cast<int>(objects_.size())) {
        selected_object_ = index;
        update();
    }
}

int Scene3DRenderer::getSelectedObject() const
{
    return selected_object_;
}

void Scene3DRenderer::setRobotPose(const std::vector<double>& joint_values)
{
    robot_joints_ = joint_values;
    update();
}

void Scene3DRenderer::renderCoordinateAxes()
{
    // 禁用光照
    glDisable(GL_LIGHTING);
    
    // 绘制坐标轴
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    
    // X轴 (红色)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    
    // Y轴 (绿色)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    
    // Z轴 (蓝色)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    
    glEnd();
    
    // 重新启用光照
    glEnable(GL_LIGHTING);
}

void Scene3DRenderer::renderObjects()
{
    for (size_t i = 0; i < objects_.size(); ++i) {
        const QVector3D& pos = objects_[i].first;
        const QColor& color = objects_[i].second;
        
        // 设置材质
        GLfloat mat_ambient[4] = { 
            static_cast<GLfloat>(color.redF() * 0.2f), 
            static_cast<GLfloat>(color.greenF() * 0.2f), 
            static_cast<GLfloat>(color.blueF() * 0.2f), 
            static_cast<GLfloat>(color.alphaF()) 
        };
        
        GLfloat mat_diffuse[4] = { 
            static_cast<GLfloat>(color.redF()), 
            static_cast<GLfloat>(color.greenF()), 
            static_cast<GLfloat>(color.blueF()), 
            static_cast<GLfloat>(color.alphaF()) 
        };
        
        GLfloat mat_specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        GLfloat mat_shininess = 50.0f;
        
        glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
        glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);
        
        // 保存当前矩阵
        glPushMatrix();
        
        // 移动到物体位置
        glTranslatef(pos.x(), pos.y(), pos.z());
        
        // 如果是选中的物体，绘制边框
        if (static_cast<int>(i) == selected_object_) {
            glDisable(GL_LIGHTING);
            glColor3f(1.0f, 1.0f, 0.0f);
            glutWireSphere(0.12f, 16, 16);
            glEnable(GL_LIGHTING);
        }
        
        // 绘制球体
        glutSolidSphere(0.1f, 16, 16);
        
        // 恢复矩阵
        glPopMatrix();
    }
}

void Scene3DRenderer::renderRobot()
{
    if (robot_joints_.empty() || robot_joints_.size() < 6) {
        return;
    }
    
    // 从关节值中提取关节角度
    float theta1 = robot_joints_[0];         // 底座旋转角度 (弧度)
    float d2 = robot_joints_[1] / 100.0f;    // 伸缩关节长度 (米)
    float theta3 = robot_joints_[2];         // 肩部关节角度 (弧度)
    float theta4 = robot_joints_[3];         // 肘部关节角度 (弧度)
    float theta5 = robot_joints_[4];         // 固定关节角度 (弧度，通常为π/2)
    float d6 = robot_joints_[5] / 100.0f;    // 末端伸缩长度 (米)
    
    // 设置材质颜色
    GLfloat arm_color[4] = { 0.7f, 0.7f, 0.7f, 1.0f };
    GLfloat joint_color[4] = { 0.4f, 0.4f, 0.4f, 1.0f };
    
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
    
    // 绘制底座
    glPushMatrix();
    glRotatef(theta1 * 180.0f / M_PI, 0.0f, 1.0f, 0.0f);
    
    // 底座圆柱
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
    glPushMatrix();
    glTranslatef(0.0f, -0.05f, 0.0f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glutSolidCylinder(0.2f, 0.1f, 20, 5);
    glPopMatrix();
    
    // 第一节臂
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.0f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glutSolidCylinder(0.05f, d2, 10, 5);
    glPopMatrix();
    
    // 肩部关节
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
    glPushMatrix();
    glTranslatef(0.0f, d2, 0.0f);
    glutSolidSphere(0.07f, 10, 10);
    glPopMatrix();
    
    // 第二节臂
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
    glPushMatrix();
    glTranslatef(0.0f, d2, 0.0f);
    glRotatef(theta3 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    glutSolidCylinder(0.04f, 0.3f, 10, 5);
    glPopMatrix();
    
    // 肘部关节
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
    glPushMatrix();
    glTranslatef(0.3f, 0.0f, 0.0f);
    glutSolidSphere(0.06f, 10, 10);
    glPopMatrix();
    
    // 第三节臂
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
    glPushMatrix();
    glTranslatef(0.3f, 0.0f, 0.0f);
    glRotatef(theta4 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    glutSolidCylinder(0.03f, 0.25f, 10, 5);
    glPopMatrix();
    
    // 腕部关节
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
    glPushMatrix();
    glTranslatef(0.25f, 0.0f, 0.0f);
    glutSolidSphere(0.05f, 10, 10);
    glPopMatrix();
    
    // 末端执行器
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
    glPushMatrix();
    glTranslatef(0.25f, 0.0f, 0.0f);
    glRotatef(theta5 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    glPushMatrix();
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glutSolidCylinder(0.02f, d6, 10, 5);
    glPopMatrix();
    
    // 末端吸盘
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
    glPushMatrix();
    glTranslatef(0.0f, -d6, 0.0f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glutSolidCylinder(0.04f, 0.02f, 10, 5);
    glPopMatrix();
    
    glPopMatrix(); // 末端执行器
    glPopMatrix(); // 第三节臂
    glPopMatrix(); // 第二节臂
    glPopMatrix(); // 底座
}

QVector3D Scene3DRenderer::screenToRay(int x, int y)
{
    // 将屏幕坐标转换为NDC坐标
    float width = this->width();
    float height = this->height();
    
    // 将像素坐标转换为[-1, 1]范围的标准化设备坐标(NDC)
    float ndcX = 2.0f * x / width - 1.0f;
    float ndcY = 1.0f - 2.0f * y / height;
    
    // 创建射线方向
    QVector4D ray_clip(ndcX, ndcY, -1.0f, 1.0f);
    
    // 创建投影矩阵
    QMatrix4x4 projection;
    projection.perspective(45.0f, width / height, 0.1f, 100.0f);
    
    // 创建视图矩阵
    QMatrix4x4 view;
    view.lookAt(
        camera_position_,
        camera_target_,
        camera_up_
    );
    
    // 计算射线方向
    QVector4D ray_eye = projection.inverted() * ray_clip;
    ray_eye = QVector4D(ray_eye.x(), ray_eye.y(), -1.0f, 0.0f);
    
    QVector4D ray_world = view.inverted() * ray_eye;
    QVector3D ray_dir(ray_world.x(), ray_world.y(), ray_world.z());
    return ray_dir.normalized();
}

bool Scene3DRenderer::rayIntersectsSphere(const QVector3D& ray_origin, const QVector3D& ray_dir, 
                                        const QVector3D& sphere_center, float sphere_radius)
{
    // 计算射线起点到球心的向量
    QVector3D oc = ray_origin - sphere_center;
    
    // 二次方程系数
    float a = QVector3D::dotProduct(ray_dir, ray_dir);
    float b = 2.0f * QVector3D::dotProduct(oc, ray_dir);
    float c = QVector3D::dotProduct(oc, oc) - sphere_radius * sphere_radius;
    
    // 判别式
    float discriminant = b * b - 4 * a * c;
    
    return discriminant > 0;
} 
#include "arm_gui/arm_control_gui.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <QOpenGLFunctions>
#include <cmath>
#include <QDebug>  // 添加调试输出支持
#include <algorithm> // 用于std::min和std::max函数
#include <limits>    // 用于处理无穷值
#include <stdexcept> // 用于异常处理

// Scene3DRenderer 实现
Scene3DRenderer::Scene3DRenderer(QWidget* parent)
    : QOpenGLWidget(parent)
    , selected_object_(-1)
    , camera_position_(0.0f, 1.0f, 3.0f)
    , camera_target_(0.0f, 0.0f, 0.0f)
    , camera_up_(0.0f, 1.0f, 0.0f)
    , yaw_(0.0f)
    , pitch_(30.0f)
    , zoom_(5.0f)
{
    // 设置焦点策略，允许通过Tab键获取焦点
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    
    // 确保窗口可见
    setMinimumSize(200, 200);
    
    // 添加默认的机械臂关节值，避免出现"关节数据为空"的错误
    robot_joints_ = {0.0, 0.0, 0.0, 0.0, M_PI/2.0, 5.0};
    
    qDebug() << "Scene3DRenderer构造函数: 初始化完成";
}

Scene3DRenderer::~Scene3DRenderer()
{
    // 获取上下文并使其成为当前上下文
    makeCurrent();
    
    // 清理OpenGL资源
    
    // 释放上下文
    doneCurrent();
    
    qDebug() << "Scene3DRenderer析构函数: 资源已释放";
}

void Scene3DRenderer::initializeGL()
{
    qDebug() << "Scene3DRenderer::initializeGL开始";
    
    // 检查OpenGL初始化
    if (!isValid()) {
        qDebug() << "ERROR: OpenGL上下文无效！";
        return;
    }
    
    try {
        // 初始化OpenGL函数
        initializeOpenGLFunctions();
        
        // 设置清除颜色
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        
        // 启用深度测试
        glEnable(GL_DEPTH_TEST);
        
        // 启用光照
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
        
        // 检查OpenGL错误
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            qDebug() << "OpenGL初始化错误:" << error;
        } else {
            qDebug() << "Scene3DRenderer::initializeGL成功";
        }
    } catch (const std::exception& e) {
        qDebug() << "Scene3DRenderer::initializeGL异常:" << e.what();
    } catch (...) {
        qDebug() << "Scene3DRenderer::initializeGL未知异常";
    }
}

void Scene3DRenderer::resizeGL(int w, int h)
{
    // 防止除以零
    if (h == 0) h = 1;
    
    qDebug() << "Scene3DRenderer::resizeGL:" << w << "x" << h;
    
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, static_cast<float>(w) / h, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void Scene3DRenderer::paintGL()
{
    try {
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
        
        // 检查OpenGL错误
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            qDebug() << "paintGL OpenGL错误:" << error;
        }
    } catch (const std::exception& e) {
        qDebug() << "paintGL异常:" << e.what();
    } catch (...) {
        qDebug() << "paintGL未知异常";
    }
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
            
            // 使用线框渲染
            glPushAttrib(GL_POLYGON_BIT);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            GLUquadricObj *quadric = gluNewQuadric();
            gluQuadricDrawStyle(quadric, GLU_LINE);
            gluSphere(quadric, 0.12f, 16, 16);
            gluDeleteQuadric(quadric);
            glPopAttrib();
            
            glEnable(GL_LIGHTING);
        }
        
        // 绘制球体
        GLUquadricObj *quadric = gluNewQuadric();
        gluQuadricDrawStyle(quadric, GLU_FILL);
        gluSphere(quadric, 0.1f, 16, 16);
        gluDeleteQuadric(quadric);
        
        // 恢复矩阵
        glPopMatrix();
    }
}

void Scene3DRenderer::renderRobot()
{
    // 检查关节数据是否有效
    if (robot_joints_.empty()) {
        qDebug() << "renderRobot: 关节数据为空";
        return;
    }
    
    // 确保关节数据完整
    if (robot_joints_.size() < 6) {
        qDebug() << "renderRobot: 关节数据不完整，期望6个值，实际" << robot_joints_.size() << "个值";
        // 补全缺少的值，确保不会因数组越界导致崩溃
        while (robot_joints_.size() < 6) {
            robot_joints_.push_back(0.0);
        }
    }
    
    try {
        // 从关节值中提取关节角度
        float theta1 = static_cast<float>(robot_joints_[0]);         // 底座旋转角度 (弧度)
        float d2 = static_cast<float>(robot_joints_[1] / 100.0f);    // 伸缩关节长度 (米)
        float theta3 = static_cast<float>(robot_joints_[2]);         // 肩部关节角度 (弧度)
        float theta4 = static_cast<float>(robot_joints_[3]);         // 肘部关节角度 (弧度)
        float theta5 = static_cast<float>(robot_joints_[4]);         // 固定关节角度 (弧度，通常为π/2)
        float d6 = static_cast<float>(robot_joints_[5] / 100.0f);    // 末端伸缩长度 (米)
        
        // 防止值无效
        if (std::isnan(theta1) || std::isnan(d2) || std::isnan(theta3) || std::isnan(theta4) || std::isnan(theta5) || std::isnan(d6)) {
            qDebug() << "renderRobot: 检测到NaN值，使用默认值";
            theta1 = 0.0f;
            d2 = 0.0f;
            theta3 = 0.0f;
            theta4 = 0.0f;
            theta5 = static_cast<float>(M_PI/2.0);
            d6 = 0.05f;
        }
        
        // 限制值的范围，确保所有参数类型一致
        const float PI_HALF_F = static_cast<float>(M_PI/2.0);
        const float PI_F = static_cast<float>(M_PI);
        
        d2 = std::max(0.0f, std::min(0.43f, d2));
        theta3 = std::max(-PI_HALF_F, std::min(PI_HALF_F, theta3));
        theta4 = std::max(0.0f, std::min(PI_F, theta4));
        d6 = std::max(0.05f, std::min(0.15f, d6));
        
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
        
        GLUquadricObj *quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.2f, 0.2f, 0.1f, 20, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 第一节臂
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, 0.0f);
        glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.05f, 0.05f, d2, 10, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 肩部关节
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
        glPushMatrix();
        glTranslatef(0.0f, d2, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere(quadric, 0.07f, 10, 10);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 第二节臂
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
        glPushMatrix();
        glTranslatef(0.0f, d2, 0.0f);
        glRotatef(theta3 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
        glPushMatrix();
        glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.04f, 0.04f, 0.3f, 10, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 肘部关节
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
        glPushMatrix();
        glTranslatef(0.3f, 0.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere(quadric, 0.06f, 10, 10);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 第三节臂
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
        glPushMatrix();
        glTranslatef(0.3f, 0.0f, 0.0f);
        glRotatef(theta4 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
        glPushMatrix();
        glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.03f, 0.03f, 0.25f, 10, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 腕部关节
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
        glPushMatrix();
        glTranslatef(0.25f, 0.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere(quadric, 0.05f, 10, 10);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 末端执行器
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, arm_color);
        glPushMatrix();
        glTranslatef(0.25f, 0.0f, 0.0f);
        glRotatef(theta5 * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
        glPushMatrix();
        glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.02f, 0.02f, d6, 10, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        // 末端吸盘
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, joint_color);
        glPushMatrix();
        glTranslatef(0.0f, -d6, 0.0f);
        glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        
        quadric = gluNewQuadric();
        if (quadric) {
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluCylinder(quadric, 0.04f, 0.04f, 0.02f, 10, 5);
            gluDeleteQuadric(quadric);
        }
        
        glPopMatrix();
        
        glPopMatrix(); // 末端执行器
        glPopMatrix(); // 第三节臂
        glPopMatrix(); // 第二节臂
        glPopMatrix(); // 底座
    } catch (const std::exception& e) {
        qDebug() << "renderRobot异常:" << e.what();
    } catch (...) {
        qDebug() << "renderRobot未知异常";
    }
    
    // 检查OpenGL错误
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        qDebug() << "renderRobot OpenGL错误:" << error;
    }
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
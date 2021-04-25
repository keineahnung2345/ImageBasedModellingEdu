
//
// Created by sway on 2021/3/11.
//
#include <iostream>
#include <GL/glew.h>

#include <fstream>
#include <sstream>
#include <string>

#ifdef  __APPLE__
#include <GLUT/glut.h>

#endif
#ifdef __linux__
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/gl.h>
#endif


#define BUFFER_OFFSET(offset) ((void*) NULL + offset)
#define NumberOf(array) (sizeof(array)/sizeof(array[0]))

GLfloat angle = 0.0;
GLfloat trans =0.0;
GLsizei n_elements;

// 顶点数组索引
GLuint VAO;

// 定义属性对应的缓冲器
enum {Vertices, Colors, Elements, NumVBOs};
GLuint buffers[NumVBOs];

void read_ply(const char* fname){
    glGenVertexArrays(1, &VAO);
    std::ifstream infile(fname);
    int VERTEX_CNT, FACE_CNT;

    std::string line;
    while (std::getline(infile, line, '\n')) {
        if(line.find("element vertex") != std::string::npos){
            VERTEX_CNT = std::stoi(line.substr(15));
        }else if(line.find("element face") != std::string::npos){
            FACE_CNT = std::stoi(line.substr(13));
        }else if(line.find("end_header") != std::string::npos){
            break;
        }
    }

    std::cout << "vertex count: " << VERTEX_CNT << std::endl;
    std::cout << "face count: " << FACE_CNT << std::endl;

    auto vert_xyzs = new GLfloat [VERTEX_CNT][3];
    auto vert_colors = new GLfloat [VERTEX_CNT][3];
    // GLfloat (*vert_xyzs)[3] = new GLfloat[VERTEX_CNT][3];
    // GLfloat (*vert_colors)[3] = new GLfloat[VERTEX_CNT][3];
    // this array stores the indices of vertices, so it must be able to represent 5706316
    // so it's type should not be ubyte, it should be uint!
    // it has to be unsigned int, not int?
    GLuint* face_indices = new GLuint [FACE_CNT * 3];
    std::string id, x, y, z, r, g, b, alpha, cnt, i1, i2, i3;

    for(size_t i = 0; i < VERTEX_CNT; ++i) {
        std::getline(infile, line, '\n');
        std::stringstream ss(line);
        ss >> x >> y >> z >> r >> g >> b >> alpha;

        // if(i % 100000 == 0){
        //     std::cout << i << std::endl;
        // }

        try{
            vert_xyzs[i][0] = std::stof(x);
            vert_xyzs[i][1] = std::stof(y);
            vert_xyzs[i][2] = std::stof(z);
            vert_colors[i][0] = std::stof(r)/255.0;
            vert_colors[i][1] = std::stof(g)/255.0;
            vert_colors[i][2] = std::stof(b)/255.0;

            if(i > VERTEX_CNT - 10){
                std::cout << vert_xyzs[i][0] << " " << vert_xyzs[i][1] << " " << vert_xyzs[i][2] << 
                    " " << vert_colors[i][0] << " " << vert_colors[i][1] << " " << vert_colors[i][2] << std::endl;
            }
        }catch(const char* msg){
            std::cout << "Exception: " << msg << std::endl;
            std::cout << x << " " << y << " " << z
                << " " << r << " " << g << " " << b << " " << alpha << std::endl;
        }
        
    }

    for(size_t i = 0; i < FACE_CNT; ++i) {
        std::getline(infile, line, '\n');
        std::stringstream ss(line);
        ss >> cnt >> i1 >> i2 >> i3;

        // if(i % 100000 == 0){
        //     std::cout << i << std::endl;
        // }

        try{
            face_indices[i*3  ] = (unsigned int)std::stoi(i1);
            face_indices[i*3+1] = (unsigned int)std::stoi(i2);
            face_indices[i*3+2] = (unsigned int)std::stoi(i3);

            if(i > FACE_CNT - 10){
                std::cout << face_indices[i*3  ] << " " << face_indices[i*3+1] << " " << face_indices[i*3+2] << std::endl;
            }
        }catch(const char* msg){
            std::cout << "Exception: " << msg << std::endl;
            std::cout << cnt << " " << i1 << " " << i2 << " " << i3 << std::endl;
        }
    }

    // std::cout << "Vertices size: " << sizeof(vert_xyzs) << std::endl; //8
    // std::cout << "Colors size: " << sizeof(vert_colors) << std::endl; //8
    // std::cout << "Elements size: " << sizeof(face_indices) << std::endl; //8
    
    n_elements = FACE_CNT*3;

    std::cout << "Vertices size: " << VERTEX_CNT*3*4 << std::endl; //68475792
    std::cout << "Colors size: " << VERTEX_CNT*3*4 << std::endl; //68475792
    std::cout << "Elements size: " << FACE_CNT*3*4 << std::endl; //33805833
    std::cout << "n_elements: " << n_elements << std::endl; //33805833

    // 绑定顶点数组
    glBindVertexArray(VAO);

    // 每个属性定义一个buffer
    glGenBuffers(NumVBOs, buffers);

    // 绑定缓冲器，并绑定定点数组数据
    // 顶点坐标
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Vertices]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vert_xyzs), vert_xyzs, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, VERTEX_CNT*3*4, vert_xyzs, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    // 颜色
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Colors]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vert_colors), vert_colors, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, VERTEX_CNT*3*4, vert_colors, GL_STATIC_DRAW);
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // 面片索引
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[Elements]);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(face_indices), face_indices, GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, FACE_CNT*3*4, face_indices, GL_STATIC_DRAW);
}

void make_objects(){

    // 创建顶点数组对象
    glGenVertexArrays(1, &VAO);
    GLfloat cubeVerts[][3]={
            {-1.0, -1.0, -1.0},
            {-1.0, -1.0, 1.0},
            {-1.0, 1.0, -1.0},
            {-1.0, 1.0, 1.0},
            {1.0, -1.0, -1.0},
            {1.0, -1.0, 1.0},
            {1.0, 1.0, -1.0},
            {1.0, 1.0,  1.0}
    };

    GLfloat cubeColors[][3]={
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 1.0},
            {0.0, 1.0, 0.0},
            {0.0, 1.0, 0.0},
            {1.0, 0.0, 0.0},
            {1.0, 0.0, 1.0},
            {1.0, 1.0, 0.0},
            {1.0, 1.0, 1.0}
    };

    GLubyte cubeIndices[]={
            0,  1,  3,  2,
            4,  6,  7,  6,
            2,  3,  7,  6,
            0,  4,  5,  1,
            0,  2,  6,  4,
            1,  5,  7,  3
    };
    n_elements = NumberOf(cubeIndices);

    std::cout << "Vertices size: " << sizeof(cubeVerts) << std::endl; //96
    std::cout << "Colors size: " << sizeof(cubeColors) << std::endl; //96
    std::cout << "Elements size: " << sizeof(cubeIndices) << std::endl; //24
    std::cout << "n_elements: " << n_elements << std::endl; //24

    // 绑定定点数组
    glBindVertexArray(VAO);

    // 每个属性定义一个buffer
    glGenBuffers(NumVBOs, buffers);

    // 绑定缓冲器，并绑定定点数组数据
    // 顶点坐标
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Vertices]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVerts), cubeVerts, GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    // 颜色
    glBindBuffer(GL_ARRAY_BUFFER, buffers[Colors]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeColors), cubeColors, GL_STATIC_DRAW);
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // 面片索引
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers[Elements]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);


}
void init(){

    glewInit();  // very important, without this statement causes segment error!!
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);  // 平滑模式

    // // 创建物体
    // make_objects();

    const char* fname = "/home/ubt/Documents/recon/office0_ascii.ply";
    read_ply(fname);
}

void display(){

    // 清除深度缓冲器和颜色缓冲器
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();
    glTranslatef(1.5f, trans, -7.0f);  // Move right and into the screen
    glRotatef(angle, 0, 1.0, 0.0);// Reset the model-view matrix

    // 3种不同的模式，分别将多边形渲染成点，线和填充的模式
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(VAO);
    // glDrawElements(GL_QUADS, n_elements, GL_UNSIGNED_BYTE, BUFFER_OFFSET(0));
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

    glutSwapBuffers();
}

void reshape(int w, int h){

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    // 透视投影
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (GLfloat)w/(GLfloat)h, 0.01, 10000.0);

    // 定义视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // 相机默认位于原点，指向z轴负方向
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void spinRotate(void){
    angle = angle +2.0;
    if (angle > 360.0) angle-=360.0;
    if(angle<-360) angle += 360.0;
    glutPostRedisplay();
}

void spinTrans(void){
    trans +=0.1;
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y){
    switch (key) {
        case 'a':
            angle+=1.0;
            glutPostRedisplay();
            break;
        case 'd':
            angle-=1.0;
            glutPostRedisplay();
            break;
        case 'q':
            glutIdleFunc(NULL);
            glutPostRedisplay();
            break;
        case 'w':
            trans += 0.05;
            glutPostRedisplay();
            break;
        case 's':
            trans -= 0.05;
            glutPostRedisplay();
            break;
        default:
            glutPostRedisplay();
            break;
    }
}

int main(int argc, char*argv[]){

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);
    glutCreateWindow(argv[0]);
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
    return 0;

}
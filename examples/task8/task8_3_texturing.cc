//
// Created by sway on 2021/4/5.
//

#include <iostream>
#include <GL/glew.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

// to read jpg
//https://stackoverflow.com/questions/2076475/reading-an-image-file-in-c-c
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#ifdef  __APPLE__
#include <GLUT/glut.h>

#endif
#ifdef __linux__
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/gl.h>
#endif


#define checkImageWidth 64
#define checkImageHeight 64
#define textureImageWidth 2048
#define textureImageHeight 2048

static GLubyte checkImage [checkImageHeight][checkImageWidth][4];
static GLubyte textureImage [textureImageWidth][textureImageHeight][3];
static std::vector<std::vector<float>> vs;
static std::vector<std::vector<float>> vts;
static std::vector<std::vector<float>> vns;
static std::vector<std::vector<std::size_t>> fs;
static GLuint texName;
GLfloat angle = 0.0;
GLfloat trans =0.0;


//床架棋盘格图像
void makeCheckImage(void){

    for (int i=0; i< checkImageHeight; i++){
        for (int j=0; j< checkImageWidth; j++){
           int c = (((i&0x8)==0)^((j&0x8))==0)*255;
           checkImage[i][j][0] = (GLubyte)c;
           checkImage[i][j][1] = (GLubyte)c;
           checkImage[i][j][2] = (GLubyte)c;
           checkImage[i][j][3] = (GLubyte)255;
        }
    }
}

void readImage(const char* img_fname){
    int width, height, bpp;
    // is bpp channel count?
    uint8_t* rgb_image = stbi_load(img_fname, &width, &height, &bpp, 0);
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;
    std::cout << "bpp: " << bpp << std::endl;
    std::cout << "rgb_image: " << sizeof(rgb_image) << std::endl;
    // for(int i = 0; i < width; ++i){
    //     for(int j = 0; j < height; ++j){
    //         for(int k = 0; k < bpp; ++k){
    //             int idx = i*height*bpp + j*bpp + k;
    //             std::cout << "(" << idx << ", " << (int)rgb_image[idx] << ") ";
    //             // textureImage[i][j][k] = rgb_image[i*height*bpp + j*bpp + k];
    //         }
    //         std::cout << std::endl;
    //     }
    //     std::cout << "============" << std::endl;
    // }
    // row major or column major?
    for(int i = 0; i < height; ++i){
        for(int j = 0; j < width; ++j){
            for(int k = 0; k < bpp; ++k){
                int idx = i*width*bpp + j*bpp + k;
                // std::cout << "(" << idx << ", " << (int)rgb_image[idx] << ") ";
                textureImage[i][j][k] = rgb_image[i*width*bpp + j*bpp + k];
            }
            // std::cout << std::endl;
        }
        // std::cout << "============" << std::endl;
    }
    stbi_image_free(rgb_image);
}

void writeImage(const char* img_fname){
    int width = 5; 
    int height = 5;
    int channels = 3;

    uint8_t* rgb_image;
    rgb_image = (uint8_t*)malloc(width*height*channels);

    // Write your code to populate rgb_image here
    for(int i = 0; i < width * height * channels; ++i){
        rgb_image[i] = i;
    }

    stbi_write_jpg(img_fname, width, height, channels, rgb_image, 100);
}

void readObj(const char* obj_fname){
    std::ifstream infile(obj_fname);
    int VERTEX_CNT, FACE_CNT;

    /*
    v -0.013002 -0.097233 0.244676
    v -0.013002 -0.109345 0.149967
    v -0.013002 -0.135492 0.169617
    vt 0.693283 0.780659
    vt 0.693283 0.783541
    vt 0.504198 0.783541
    vn 0.692600 0.721300 0.000000
    vn -0.692600 0.721300 -0.000000
    vn 0.884700 0.465000 -0.033500
    s off
    f 3128/1/1 3152/2/1 3153/3/1
    f 3153/3/1 3129/4/1 3128/1/1
    f 3134/5/2 3154/6/2 3155/7/2
    */
    /*
    v x y z
    vt u v [w]
    vn x y z
    f v1[/vt1][/vn1] v2[/vt2][/vn2] v3[/vt3][/vn3]
    */
    std::string line;
    std::string discard, x, y, z, tx, ty, vnx, vny, vnz;
    std::vector<std::string> ftokens = {"", "", ""};
    while (std::getline(infile, line, '\n')) {
        std::stringstream ss(line);
        if(line.find("v ") != std::string::npos){
            ss >> discard >> x >> y >> z;
            vs.push_back({std::stof(x), std::stof(y), std::stof(z)});
        }else if(line.find("vt ") != std::string::npos){
            ss >> discard >> tx >> ty;
            vts.push_back({std::stof(tx), std::stof(ty)});
        }else if(line.find("vn ") != std::string::npos){
            ss >> discard >> vnx >> vny >> vnz;
            vns.push_back({std::stof(vnx), std::stof(vny), std::stof(vnz)});
        }else if(line.find("f ") != std::string::npos){
            ss >> discard >> ftokens[0] >> ftokens[1] >> ftokens[2];
            // tokenize f1, f2, f3 by '/'
            std::size_t pos;
            std::string delimiter = "/";
            std::vector<std::size_t> f;
            for(std::size_t i = 0; i < 3; ++i){
                while(true){
                    pos = ftokens[i].find(delimiter);
                    //works even if pos is string::npos
                    f.push_back(std::stoi(ftokens[i].substr(0, pos)));
                    if(pos == std::string::npos) break;
                    //pos+1 equals to 0, so the line below can't handle this situation
                    ftokens[i].erase(0, pos+delimiter.length());
                }
            }
            
            fs.emplace_back(f);
        }
    }

    std::cout << "read complete" << std::endl;
    std::cout << "v: " << vs.size() << " * " << vs[0].size() << std::endl;
    std::cout << "vt: " << vts.size() << " * " << vts[0].size() << std::endl;
    std::cout << "vn: " << vns.size() << " * " << vns[0].size() << std::endl;
    std::cout << "f: " << fs.size() << " * " << fs[0].size() << std::endl;
}

void init(void){

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);  // 开始深度测试

    // makeCheckImage();
    const char* img_fname = "/home/ubt/Documents/recon/ImageBasedModellingEdu/examples/task8/three.js/models/obj/cerberus/Cerberus_A.jpg";
    // const char* img_fname = "/home/ubt/Documents/recon/write.jpg";
    readImage(img_fname);
    // const char* wimg_fname = "/home/ubt/Documents/recon/write.jpg";
    // writeImage(wimg_fname);
    const char* obj_fname = "/home/ubt/Documents/recon/ImageBasedModellingEdu/examples/task8/three.js/models/obj/cerberus/Cerberus.obj";
    readObj(obj_fname);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // 开启像素缓冲区

    /**
     * 生成纹理的过程
     * 1. 生成纹理名称
     * 2. 初次把纹理对象绑定到纹理数据上，包括图像数组和纹理属性
     * 3. 绑定和重新绑定纹理对象
     */

    // 生成纹理名称，任何无符号的整数都可以用来表示纹理对象的名称
    glGenTextures(1, &texName);

    // 创建和使用纹理初次绑定时，OPenGL会创建一个新的纹理对象，并把纹理图像和纹理属性设置为默认值
    glBindTexture(GL_TEXTURE_2D, texName);

    // 设定当纹理坐标超出1的时候对纹理进行重复
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // 指定纹理图像在放大和缩小时候的处理方式
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // 定义二维纹理， 图像的像素格式为rgba
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth, checkImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, checkImage);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImageWidth, textureImageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, textureImage);
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

void drawTwoQuads(void){
    // 开始绘制四边形
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0); glVertex3f(-2.0, -1.0, 0.0);
    glTexCoord2f(0.0, 1.0); glVertex3f(-2.0, 1.0, 0.0);
    glTexCoord2f(1.0, 1.0); glVertex3f(0.0, 1.0, 0.0);
    glTexCoord2f(1.0, 0.0); glVertex3f(0.0, -1.0, 0.0);

    glTexCoord2f(0.0, 0.0); glVertex3f(1.0, -1.0, 0.0);
    glTexCoord2f(0.0, 1.0); glVertex3f(1.0, 1.0, 0.0);
    glTexCoord2f(1.0, 1.0); glVertex3f(2.41421, 1.0, -1.41421);
    glTexCoord2f(1.0, 0.0); glVertex3f(2.41421, -1.0, -1.41421);
    glEnd();
}

void drawObj(){
    // std::cout << "v: " << vs.size() << " * " << vs[0].size() << std::endl;
    // std::cout << "vt: " << vts.size() << " * " << vts[0].size() << std::endl;
    // std::cout << "vn: " << vns.size() << " * " << vns[0].size() << std::endl;
    // std::cout << "f: " << fs.size() << " * " << fs[0].size() << std::endl;

    glBegin(GL_TRIANGLES); //?

    std::size_t vix, vtix, vnix;
    for(std::size_t i = 0; i < fs.size(); ++i){
        for(std::size_t j = 0; j < 3; ++j){
            // std::cout << "fs[" << i << "]: " << fs[i].size() << std::endl;
            //the indices start from 1?
            vix = fs[i][j*3+0]-1;
            vtix = fs[i][j*3+1]-1;
            vnix = fs[i][j*3+2]-1;
            // std::cout << "vertex texture index: " << vtix << "/" << vts.size() << std::endl;
            // std::cout << "vertex index: " << vix << "/" << vs.size() << std::endl;
            glTexCoord2f(vts[vtix][0], vts[vtix][1]); 
            glVertex3f(vs[vix][0], vs[vix][1], vs[vix][2]);
        }
    }

    glEnd();
}

void display(void){

    // 每次渲染都要清除之前的缓冲器
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 开启纹理渲染
    glEnable(GL_TEXTURE_2D);

    // 将绘图模式设置为 GL_REPLACE 表示经过纹理贴图后，多边形的颜色完全来自与纹理图像
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    // 绑定texName的纹理
    glBindTexture(GL_TEXTURE_2D, texName);

    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();
    glTranslatef(1.5f, trans, -7.0f);  // Move right and into the screen
    glRotatef(angle, 0, 1.0, 0.0);// Reset the model-view matrix


    // drawTwoQuads();
    drawObj();

    glFlush();
    glutSwapBuffers();
    glDisable(GL_TEXTURE_2D);

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
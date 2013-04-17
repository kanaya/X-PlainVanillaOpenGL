/*
    quatsample --- A sample program for the quaternion library.
    Copyright (C) 2004-2013 Ichiroh Kanaya

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <stdlib.h>
#include <GLUT/glut.h>
#include "quat.h"

#define FALSE 0
#define TRUE  1

#define WINDOW_SIZE 512

#define FOVY 20.0

#define R 0.8

#define ROOT_2_INV 0.70710678118654752440

#define APP_QUIT 1

#define SQR(x) ((x) * (x))

static const GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
static const GLfloat blue[]  = { 0.0, 0.0, 1.0, 1.0 };
static const GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };

static int scaling = FALSE;
static int begin_x, begin_y;
static quat curr, last;
static GLfloat scale_factor = 1.0;

static int width = WINDOW_SIZE, height = WINDOW_SIZE;

static GLfloat project_to_sphere(GLfloat x, GLfloat y) {
  GLfloat z;
  GLfloat d_sqr, d;
  
  d_sqr = SQR(x) + SQR(y);
  d = sqrt(d_sqr);
  if (d < R) {
    z = sqrt(2.0 * SQR(R) - d_sqr);
  }
  else {
    z = SQR(R) / d;
  }
  return z;
}

static void simulate_trackball(quat *q, GLfloat p1x, GLfloat p1y, GLfloat p2x, GLfloat p2y) {
  if (p1x == p2x && p1y == p2y) { 
    quat_identity(q);
  }
  else {
    quat p1, p2, a, d;
    float p1z, p2z;
    float s, t;
    
    p1z = project_to_sphere(p1x, p1y);
    quat_assign(&p1, 0.0, p1x, p1y, p1z);
    
    p2z = project_to_sphere(p2x, p2y);
    quat_assign(&p2, 0.0, p2x, p2y, p2z);
		
    quat_mul(&a, &p1, &p2);
    
    a.w = 0.0;
    s = quat_norm(&a);
    quat_div_real(&a, &a, s);
    
    quat_sub(&d, &p1, &p2);
    t = quat_norm(&d) / (2.0 * R * ROOT_2_INV);
    if (t > 1.0) t = 1.0;
    quat_assign(q, cos(asin(t)), a.x * t, a.y * t, a.z * t);
  }
}

static void create_rotation_matrix(GLfloat m[4][4], const quat *q) {
  m[0][0] = 1.0 - 2.0 * (q->y * q->y + q->z * q->z);
  m[0][1] =       2.0 * (q->x * q->y - q->z * q->w);
  m[0][2] =       2.0 * (q->z * q->x + q->w * q->y);
  m[0][3] = 0.0;
  m[1][0] =       2.0 * (q->x * q->y + q->z * q->w);
  m[1][1] = 1.0 - 2.0 * (q->z * q->z + q->x * q->x);
  m[1][2] =       2.0 * (q->y * q->z - q->w * q->x);
  m[1][3] = 0.0;
  m[2][0] =       2.0 * (q->z * q->x - q->w * q->y);
  m[2][1] =       2.0 * (q->y * q->z + q->x * q->w);
  m[2][2] = 1.0 - 2.0 * (q->y * q->y + q->x * q->x);
  m[2][3] = 0.0;
  m[3][0] = 0.0;
  m[3][1] = 0.0;
  m[3][2] = 0.0;
  m[3][3] = 1.0;
}

#define L 0.5
static void draw_frame(void) {
  static GLfloat vdata[8][3] = {
    { -L, -L, -L }, { L, -L, -L }, { L, L, -L }, { -L, L, -L },
    { -L, -L, L }, { L, -L, L }, { L, L, L }, { -L, L, L } };
  static GLint tindices[6][4] = {
    { 0, 1, 2, 3 }, { 0, 1, 5, 4 }, { 3, 0, 4, 7 },
    { 1, 2, 6, 5 }, { 2, 3, 7, 6 },{ 5, 4, 7, 6 } };
  int i;
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, blue);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
  for (i = 0; i < 3; ++i) {
    glBegin(GL_QUADS);
    glNormal3fv(&vdata[tindices[i][0]][0]);
    glVertex3fv(&vdata[tindices[i][0]][0]);
    glNormal3fv(&vdata[tindices[i][1]][0]);
    glVertex3fv(&vdata[tindices[i][1]][0]);
    glNormal3fv(&vdata[tindices[i][2]][0]);
    glVertex3fv(&vdata[tindices[i][2]][0]);
    glNormal3fv(&vdata[tindices[i][3]][0]);
    glVertex3fv(&vdata[tindices[i][3]][0]);
    glEnd();
  }
}

#define X (0.525731112119133606 / 2.0)
#define Z (0.850650808352039932 / 2.0)
static void draw_object(void) {
  static GLfloat vdata[12][3] = {
    { -X, 0.0, Z }, { X, 0.0, Z }, { -X, 0.0, -Z },
    { X, 0.0, -Z }, { 0.0, Z, X }, { 0.0, Z, -X },
    { 0.0, -Z, X }, { 0.0, -Z, -X }, { Z, X, 0.0 },
    { -Z, X, 0.0 }, { Z, -X, 0.0 }, { -Z, -X, 0.0 } };
  static GLint tindices[20][3] = {
    { 0, 4, 1 }, { 0, 9, 4 }, { 9, 5, 4 }, { 4, 5, 8 },
    { 4, 8, 1 }, { 8, 10, 1 }, { 8, 3, 10 }, { 5, 3, 8 },
    { 5, 2, 3 }, { 2, 7, 3 }, { 7, 10, 3 }, { 7, 6, 10 },
    { 7, 11, 6 }, { 11, 0, 6 }, { 0, 1, 6 }, { 6, 1, 10 }, 
    { 9, 0, 11 }, { 9, 11, 2 }, { 9, 2, 5 }, { 7, 2, 11 } };
  int i;
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, white);
  for (i = 0; i < 20; ++i) {
    glBegin(GL_TRIANGLES);
    glNormal3fv(&vdata[tindices[i][0]][0]);
    glVertex3fv(&vdata[tindices[i][0]][0]);
    glNormal3fv(&vdata[tindices[i][1]][0]);
    glVertex3fv(&vdata[tindices[i][1]][0]);
    glNormal3fv(&vdata[tindices[i][2]][0]);
    glVertex3fv(&vdata[tindices[i][2]][0]);
    glEnd();
  }
}

static void draw(void) {
  static int first_call = TRUE;
  static GLint display_list = 0;
	
  if (first_call) {
    first_call = FALSE;
    display_list = glGenLists(1);
    glNewList(display_list, GL_COMPILE_AND_EXECUTE);
    draw_frame();
    draw_object();
    glEndList();
  }
  else {
    glCallList(display_list);
  }
}

static void display_func(void) {
  static GLfloat v[4][4] = {
    { 1.0, 0.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0, 0.0 },
    { 0.0, 0.0, -1.0, 0.0 },
    { 0.0, 0.0, 0.0, 1.0 } };

  GLfloat m[4][4];
  int i, j;
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glMultMatrixf(&v[0][0]);

  glPushMatrix();
  create_rotation_matrix(m, &curr);
  glScalef(scale_factor, scale_factor, scale_factor);
  glMultMatrixf(&m[0][0]);
  draw();
  glPopMatrix();
  glutSwapBuffers();
}

static void reshape_func(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(FOVY, (GLfloat)w/(GLfloat)h, 1.0, -100.0);
  glMatrixMode(GL_MODELVIEW);
  width = w;
  height = h;
}

static void app_menu_func(int menu) {
  switch (menu) {
  case APP_QUIT:
    exit(0);
    break;
  default:
    break;
  }
}

static void menu_func(int dummy0) {
  return;
}

static void keyboard_func(unsigned char key, int dummy1, int dummy2) {
  switch (key) {
  case 'q':
  case 'Q':
    app_menu_func(APP_QUIT);
    break;
  default:
    break;
  }
}

static void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    begin_x = x;
    begin_y = y;
    if (glutGetModifiers() & GLUT_ACTIVE_SHIFT) { 
      scaling = TRUE;
    }
    else {
      scaling = FALSE;
    }
  }
}

static void motion(int x, int y) {
  static int count = 0;
  
  if (scaling) {
    scale_factor = scale_factor * (1.0 + (((float) (begin_y - y)) / height));
    begin_x = x;
    begin_y = y;
    glutPostRedisplay();
    return;
  }
  else {
    quat t;
    
    simulate_trackball(&last, (2.0 * begin_x - width) / width, (height - 2.0 * begin_y) / height, (2.0 * x - width) / width, (height - 2.0 * y) / height);
    begin_x = x;
    begin_y = y;
    quat_mul(&t, &last, &curr);
    curr = t;
    
    if (++count % 97) {
      GLfloat n;
      
      n = quat_norm(&curr);
      quat_div_real(&curr, &curr, n);
    }
    glutPostRedisplay();
  }
}

static void init_glut_menu(void) {
  int app_menu;
  
  app_menu = glutCreateMenu(app_menu_func);
  glutAddMenuEntry("Quit (Q)", APP_QUIT);
  
  glutCreateMenu(menu_func);
  glutAddSubMenu("Application", app_menu);
  
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}

static void init_glut(int *argc, char **argv) {
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
  glutInitWindowPosition(100, 100);
  glutCreateWindow(argv[0]);
  glutKeyboardFunc(keyboard_func);
  glutDisplayFunc(display_func);
  glutReshapeFunc(reshape_func);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutSetCursor(GLUT_CURSOR_RIGHT_ARROW);
  init_glut_menu();
}

static void init_gl(void) {
  static GLfloat light1[] = { 0.0, 0.0, -10.0, 1.0 };
	
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  
  glLightfv(GL_LIGHT0, GL_DIFFUSE, black);
  glEnable(GL_LIGHT0);
  
  glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
  glLightfv(GL_LIGHT1, GL_POSITION, light1);
  glEnable(GL_LIGHT1);
  
  simulate_trackball(&curr, 0.0, 0.0, 0.0, 0.0);
}

int main(int argc, char **argv) {
  init_glut(&argc, argv);
  init_gl();
  glutMainLoop();
  return 0;
}

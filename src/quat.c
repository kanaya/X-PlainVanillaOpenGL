/*
    quat --- A quaternion library.
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
#include "quat.h"

/* a = 0 */
void quat_zero(quat *a) {
  a->w = a->x = a->y = a->z = 0.0;
}

/* a = 1 */
void quat_identity(quat *a) {
  a->w = 1.0;
  a->x = a->y = a->z = 0.0;
}

/* a = (w, x, y, z) */
void quat_assign(quat *a, float w, float x, float y, float z) {
  a->w = w;
  a->x = x;
  a->y = y;
  a->z = z;
}

/* a = b + c */
void quat_add(quat *a, const quat *b, const quat *c) {
  a->w = b->w + c->w;
  a->x = b->x + c->x;
  a->y = b->y + c->y;
  a->z = b->z + c->z;
}

/* a = b - c */
void quat_sub(quat *a, const quat *b, const quat *c) {
  a->w = b->w - c->w;
  a->x = b->x - c->x;
  a->y = b->y - c->y;
  a->z = b->z - c->z;
}

/* a = b * c */
void quat_mul(quat *a, const quat *b, const quat *c) {
  a->w = b->w * c->w - b->x * c->x - b->y * c->y - b->z * c->z;
  a->x = b->w * c->x + b->x * c->w - b->y * c->z + b->z * c->y;
  a->y = b->w * c->y + b->x * c->z + b->y * c->w - b->z * c->x;
  a->z = b->w * c->z - b->x * c->y + b->y * c->x + b->z * c->w;
}

/* a = s * b */
void quat_mul_real(quat *a, float s, const quat *b) {
  a->w = s * b->w;
  a->x = s * b->x;
  a->y = s * b->y;
  a->z = s * b->z;
}

/* a = b / s */
void quat_div_real(quat *a, const quat *b, float s) {
  a->w = b->w / s;
  a->x = b->x / s;
  a->y = b->y / s;
  a->z = b->z / s;
}

/* ||a||^2 */
float quat_norm_sqr(const quat *a) {
  return a->w * a->w + a->x * a->x + a->y * a->y + a->z * a->z;
}

/* ||a|| */
float quat_norm(const quat *a) {
  return sqrt(quat_norm_sqr(a));
}


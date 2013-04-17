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


#ifndef __QUAT_H
#define __QUAT_H

struct QUAT {
  float w, x, y, z;
};
typedef struct QUAT quat;

/* a = 0 */
void quat_zero(quat *a);

/* a = 1 */
void quat_identity(quat *a);

/* a = (w, x, y, z) */
void quat_assign(quat *a, float w, float x, float y, float z);

/* a = b + c */
void quat_add(quat *a, const quat *b, const quat *c);

/* a = b - c */

void quat_sub(quat *a, const quat *b, const quat *c);

/* a = b * c */
void quat_mul(quat *a, const quat *b, const quat *c);

/* a = s * b */
void quat_mul_real(quat *a, float s, const quat *b);

/* a = b / s */
void quat_div_real(quat *a, const quat *b, float s);

/* ||a||^2 */
float quat_norm_sqr(const quat *a);

/* ||a|| */
float quat_norm(const quat *a);

#endif

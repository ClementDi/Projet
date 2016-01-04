/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#ifndef TERRAIN_HPP
#define TERRAIN_HPP



#include "../lib/mesh/mesh.hpp"
#include "../lib/opengl/mesh_opengl.hpp"

namespace cpe
{


/** A derivated class for mesh_basic giving public access to modifier */
class terrain
{

 public:
    cpe::mesh mesh_terrain;
    cpe::mesh_opengl mesh_terrain_opengl;
    cpe::mesh  plane_mesh_constructor(float xmin,float xmax, float ymin,float ymax,int Nj,int Ni);
    void init();



};
}

#endif

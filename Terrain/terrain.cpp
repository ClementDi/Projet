#include "terrain.hpp"

namespace cpe
{





void terrain::init(){
    mesh_terrain=plane_mesh_constructor(-1,1,-1,1,10,10);
    mesh_terrain.fill_empty_field_by_default();
    mesh_terrain.fill_normal();
    mesh_terrain_opengl.fill_vbo(mesh_terrain);
}






cpe::mesh  terrain::plane_mesh_constructor(float xmin,float xmax, float ymin,float ymax,int Nj,int Ni)
 {
    cpe::mesh ret;
    //question 2
float i_max=Ni-(2*Ni/5);
float i_min=2*Ni/5;
float j_max=Nj-(1*Nj/5);
float j_min=1*Nj/5;

    for (int i=0;i<Ni;i++)
    {
        for(int j=0;j<Nj;j++)
        {
            float x =(xmax-xmin)*float(j)+xmin;
            float y =(ymax-ymin)*float(i)+ymin;
            ret.add_vertex(vec3(x,y,0));
        }

    }

    for (int i=0;i<Ni-1;i++)
    {
        for(int j=0;j<Nj-1;j++)
        {
            //triangle 1
            int t11=j+Nj*i;// en j,i
            int t12=j+Nj*(i+1);// en j,i+1
            int t13=j+1+Nj*(i);// en j+1,i
            //triangle 2
            int t21=j+1+Ni*i;
            int t23=j+1+Ni*(i+1);
            int t22=j+Ni*(i+1);
            ret.add_triangle_index({t11,t12,t13});
            ret.add_triangle_index({t21,t22,t23});

         }
    }

    for (int i=0;i<Ni;i++)
    {
        for(int j=0;j<Nj;j++)
          {

            float col=float(i%(Ni))/(Ni-1);
            float lign=float(j%(Nj))/(Nj-1);
            ret.add_texture_coord({lign,col});
          }
    }
    ret.fill_empty_field_by_default();
    ret.fill_color(vec3(1.0f,1.0f,1.0f));
    //ret.transform_apply_auto_scale_and_center();

    return ret;
}





















}

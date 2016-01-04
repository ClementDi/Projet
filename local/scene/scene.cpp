

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"
#include "../../lib/mesh/mesh_io.hpp"
#include "../../lib/common/error_handling.hpp"


#include <cmath>
#include <string>
#include <sstream>



using namespace cpe;



static cpe::mesh build_ground(float const L,float const h);
static cpe::mesh build_sphere(float radius,vec3 center);
static cpe::mesh build_plane();


void scene::load_scene()
{
    time_integration.restart();
    delta_t=0.1f;
    divergence=false;

    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");

    texture_cloth = load_texture_file("data/cloth.png");
    texture_ground = load_texture_file("data/wood_texture.png");

    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(1.0f , -1.101f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);

    //*****************************************//
    // Sphere
    //*****************************************//
    mesh_sphere = build_sphere(0.4f , {0.5f,0.05f,-1.1f});
    mesh_sphere.fill_empty_field_by_default();
    mesh_sphere_opengl.fill_vbo(mesh_sphere);

    //*****************************************//
    // Build cloth
    //*****************************************//
    mesh_cloth=mesh_parametric_cloth();
    mesh_cloth.set_plane_xy_unit(50,50);
    mesh_cloth.fill_empty_field_by_default();
    mesh_cloth_opengl.fill_vbo(mesh_cloth);


    // initialisation des parametres
     Kdirect= 25.0f;
     Kdiag=10.0f;
     Kdouble=10.0f;
     Kw=0.006;
     dt=delta_t;
     mu=0.2f;
     x_wind = 1.0f;
     y_wind = 0.0f;
     z_wind = 0.0f;

     //ecrire fonction de la forme
     mesh_cloth.rate_value(Kdirect,Kdiag,Kdouble,Kw,dt, mu,x_wind,y_wind,z_wind);
     //;et un bouleen de chagement





     mesh_plane=build_plane();
     mesh_plane.fill_empty_field_by_default();
     mesh_plane.fill_normal();
     mesh_plane_opengl.fill_vbo(mesh_plane);



     //test
        Terre.init();






















}



void scene::draw_scene()
{

    setup_shader_mesh(shader_mesh);

    // draw the ground
    glBindTexture(GL_TEXTURE_2D,texture_ground);                                                       PRINT_OPENGL_ERROR();
    mesh_ground_opengl.draw();
    // draw the sphere
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    //mesh_sphere_opengl.draw();

    mesh_plane_opengl.draw();
    //try numerical integration (stop computation if divergence)
    try
    {
        if(divergence==false && time_integration.elapsed() > 1)
        {
            //ajouter un ifis changed
            mesh_cloth.rate_value(Kdirect,Kdiag,Kdouble,Kw,dt, mu,x_wind,y_wind,z_wind);
            if (dt>0 && dt < 0.15){

                delta_t=dt;
            }
            // compute-force / time integration
            mesh_cloth.update_force();
            mesh_cloth.integration_step(delta_t);

            // re-compute normals
            mesh_cloth.fill_normal();

            // update opengl container
            mesh_cloth_opengl.update_vbo_vertex(mesh_cloth);
            mesh_cloth_opengl.update_vbo_normal(mesh_cloth);

            time_integration.restart();
        }
    }
    catch(exception_divergence const& e)
    {
        if(divergence==false)
        {
            std::cout<<"\n\nDivergence, time integration stoped"<<std::endl;
            divergence = true;
        }
    }


    Terre.mesh_terrain_opengl.draw();
}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();

}


scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}

static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex({ -L/2.0f, -L , h });
    m.add_vertex({ -L/2.0f,  L , h });
    m.add_vertex({ 1.5f*L,  L , h });
    m.add_vertex({ 1.5f*L, -L , h });

    m.add_texture_coord({  0.0f ,  0.0f });
    m.add_texture_coord({  0.0f ,  1.0f });
    m.add_texture_coord({  1.0f ,  1.0f });
    m.add_texture_coord({  1.0f ,  0.0f });

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});


    return m;
}

static cpe::mesh build_sphere(float radius,vec3 center)
{
    mesh m;
    m.load("data/sphere.off");
    m.transform_apply_scale(radius);
    m.transform_apply_translation(center);
    return m;
}

static cpe::mesh build_plane()
{
    mesh m;
    m.load("data/plane.obj");
    m.transform_apply_auto_scale_and_center();
    m.transform_apply_rotation(vec3(1.0f,0.0f,0.0f),M_PI/2);
    return m;
}






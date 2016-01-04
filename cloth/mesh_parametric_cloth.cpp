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

#include "mesh_parametric_cloth.hpp"

#include "../lib/common/error_handling.hpp"
#include <cmath>

namespace cpe
{


void mesh_parametric_cloth::rate_value(float Kdirect_s, float Kdiag_s, float Kdouble_s, float Kw_s, float dt_s,float mu_s,float x_wind_s,float y_wind_s,float z_wind_s){

    Kdirect=Kdirect_s;
    Kdiag=Kdiag_s;
    Kdouble=Kdouble_s;
    Kw=Kw_s;
    dt=dt_s;//cas special
    mu=mu_s;
    x_wind=x_wind_s;
    y_wind=y_wind_s;
    z_wind=z_wind_s;


}


void mesh_parametric_cloth::update_force()
{

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(force_data.size()) == Nu*Nv , "Error of size");
    //wind direction
    vec3 uw=vec3( x_wind,y_wind,z_wind);
    uw= normalized(uw);

    //Gravity and wind
    static vec3 const g (0.0f,0.0f,-9.81f);
    vec3 const g_normalized = g/N_total;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {

            force(ku,kv) = g_normalized;


            vec3 n=this->normal(ku,kv);
            force(ku,kv) += Kw*dot(n,uw)*n;
        }
    }

    //*************************************************************//
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//
    //*************************************************************//
    float L01= 1.0f/(Nu-1);
    float L02= sqrt(2)/(Nu-1);
    float L03= 2.0f/(Nu-1);

    for(int ku=0 ; ku<Nu; ++ku)
    {
        for(int kv=0 ; kv<Nv; ++kv)
        {

            vec3 const& p = vertex(ku,kv);
            std::vector<vec3> pvd;
            std::vector<vec3> pvdiag;
            std::vector<vec3> pvb;

            /** cas general **/
            //voisin directs
            if( kv < Nv-1) {
                pvd.push_back(vertex(ku,kv+1));
            }
            if( kv > 0) {
                pvd.push_back(vertex(ku,kv-1));
            }
            if( ku < Nu-1) {
                pvd.push_back(vertex(ku+1,kv));
            }
            if( ku > 0) {
                pvd.push_back(vertex(ku-1,kv));
            }
            //voisin diagonale
            if( ku< Nu -1 && kv < Nv-1 ) {
                pvdiag.push_back(vertex(ku+1,kv+1));
            }
            if( ku < Nu-1 && kv >0 ) {
                pvdiag.push_back(vertex(ku+1,kv-1));
            }
            if( ku > 0 && kv > 0 ) {
                pvdiag.push_back(vertex(ku-1,kv-1));
            }
            if( ku>0 && kv< Nu -1) {
                pvdiag.push_back(vertex(ku-1,kv+1));
            }

            //voisin bend
            if( ku < Nu -2) {
                pvb.push_back(vertex(ku+2,kv));
            }
            if( ku >1 ) {
                pvb.push_back(vertex(ku-2,kv));

            }
            if( kv < Nv -2) {
                pvb.push_back(vertex(ku,kv+2));
            }
            if( kv >1 ) {
                pvb.push_back(vertex(ku,kv-2));
            }



            for (int i=0; i<int(pvd.size());i++) {
                vec3 u=p-pvd[i];
                force(ku,kv)  += (Kdirect *(L01 - norm(u)) *u) /norm(u);
            }
            for (int i=0; i<int (pvdiag.size());i++) {
                vec3 u=p-pvdiag[i];
                force(ku,kv)  +=( Kdiag *(L02 - norm(u))*u) /norm(u);
            }
            for (int i=0; i<int (pvb.size());i++) {
                vec3 u=p-pvb[i];
                force(ku,kv)  +=( Kdouble *(L03 - norm(u))*u) /norm(u);
            }


            // on bloque les angles
           if ((ku == 0 && kv == 0 ) || (ku == 0 && kv == Nv -1)) {
            //if ((ku == Nu-1 && kv == Nv-1 )||(ku == Nu-1 && kv == 0 )||(ku == 0 && kv == 0 ) || (ku == 0 && kv == Nv -1)) {

                force(ku,kv) = vec3(0.0f,0.0f, 0.0f);
            }
        }
    }

}

void mesh_parametric_cloth::integration_step(float const dt)
{
    ASSERT_CPE(speed_data.size() == force_data.size(),"Incorrect size");
    ASSERT_CPE(static_cast<int>(speed_data.size()) == size_vertex(),"Incorrect size");


    int const Nu = size_u();//nombre de ressort
    int const Nv = size_v();//nombre de ressort
    //*************************************************************//
    // TO DO: Calculer l'integration numerique des positions au cours de l'intervalle de temps dt.
    //*************************************************************//

    //security check (throw exception if divergence is detected)
    static float const LIMIT=30.0f;
 for(int ku=0 ; ku<Nu; ++ku)
    {
        for(int kv=0; kv<Nv; ++kv)
        {
            vec3 & p = vertex(ku,kv);
            speed(ku,kv)=(1-mu*dt)*speed(ku,kv)+ dt*force(ku,kv);//gg c'est beau
            p = p + dt*speed(ku,kv);

            // collision avec le sol
            if (p.z() < -1.101f)// hauteur du sol
            {
                p.z()= -1.100f;
                force(ku,kv).z() = 0.0f;
                speed(ku,kv).z() = 0.0f;
            }
            // collision avec la sphere
            vec3 centre_sphere = {0.5f,0.05f,-1.1f};
            vec3 normale= p-centre_sphere;
            float rayon =0.4f;
            if (norm(normale)<rayon) {

                p= centre_sphere + normale/norm(normale) * rayon;
                //on annule la composante normale de la force  et de la vitesse

                speed(ku,kv) -= dot (speed(ku,kv),normale/norm(normale)) * normale;
                force(ku,kv) -= dot (force(ku,kv),normale/norm(normale)) * normale;




                //on annule la composante normale de la force  et de la vitesse

            }

        }
    }

}

void mesh_parametric_cloth::set_plane_xy_unit(int const size_u_param,int const size_v_param)
{
    mesh_parametric::set_plane_xy_unit(size_u_param,size_v_param);

    int const N = size_u()*size_v();
    speed_data.resize(N);
    force_data.resize(N);
}

vec3 const& mesh_parametric_cloth::speed(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3& mesh_parametric_cloth::speed(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3 const& mesh_parametric_cloth::force(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}

vec3& mesh_parametric_cloth::force(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}


}

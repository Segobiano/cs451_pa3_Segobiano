//------------------------------------------------------------------------------
//  Copyright 2007-2014 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------


#pragma once

#include "objReader.h"
#include "model.h"
#include "Library.hpp" //motion capture library
#include <list>
#include <float.h>
#include "pose_utils.hpp"

using namespace std;

double raid(double d){

	return (d*PI / 180);
}




//-----------------------------------------------------------------------------
// INPUTS
list<string> input_filenames;
string mocap_dir; //motion capture dir

//-----------------------------------------------------------------------------
// Intermediate data
list<model> models; //NOTE: only the first model of the list is used in this code
float R=0;          //radius
Point3d COM;        //center of mass

int currentMotion=0; //current motion dataset, used to animate skeleton
int currentFrame=0;  //current frame in the dataset, used to animate skeleton

//-----------------------------------------------------------------------------

/////------ PA3 Variables START-----

bool BindToSkin = false;       // initially the skeleton is not binded to the skin (mesh)

Character::Pose BindingPose;   // at this pose that the skeleton binds to the skin
vector< vector<double> > SkinningWeights; //weights for each vertex and each bone
                                          //there are N vector<double>, where N is the number of vertices in the mesh
                                          //there are K double in vector<double>, where K is the number of bones in skeleton 

vector< vector<Vector3d> > BoneSpaceCoordinates; //local coordinates for each vertex in bone subspace
                                                //there are N vector<Point3d>, where N is the number of vertices in the mesh
                                                //there are K double in vector<Point3d>, where K is the number of bones in skeleton 
vector<Vector3d> BSC;
vector<int> BoneIndex;
////------ PA3 Variables END-----

//-----------------------------------------------------------------------------

/////------ PA3 TODOs START-----

//TODO: implement this function to setup the binding pose.
//      See details below
void setupBindingPose();

//TODO: implement this function to bind the skeleton to the skin.
//      See details below
void bind2skin();

//TODO: skeleton-subspace deformation. perform SSD 
void SSD();

/////------ PA3 TODOs END-----

//-----------------------------------------------------------------------------
bool readfromfile();
void computeCOM_R();

//-----------------------------------------------------------------------------
bool parseArg(int argc, char ** argv)
{
    for(int i=1;i<argc;i++){
        if(argv[i][0]=='-')
        {
			if (string(argv[i]) == "-mocap")      mocap_dir = argv[++i];
			else
				return false; //unknown
        }
        else{
            input_filenames.push_back(argv[i]);
        }
    }

    return true;
}

void printUsage(char * name)
{
    //int offset=20;
    cerr<<"Usage: "<<name<<" [options] -mocap dir *.obj \n"
        <<"options:\n\n";
    cerr<<"\n-- Report bugs to: Jyh-Ming Lien jmlien@gmu.edu"<<endl;
}

//-----------------------------------------------------------------------------

bool readfromfiles()
{
	if (input_filenames.empty())
	{
		cerr << "! Error: No input model" << endl;
		return false;
	}

	if (mocap_dir.empty())
	{
		cerr << "! Error: No input motion capture data" << endl;
		return false;
	}

	//read obj model
    long vsize=0;
    long fsize=0;

    uint id=0;
    for(list<string>::iterator i=input_filenames.begin();i!=input_filenames.end();i++,id++){
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Start reading "<<*i<<endl;
        model m;
        if(!m.build(*i)) continue;
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Done reading "<<m.v_size<<" vertices and "<<m.t_size<<" facets"<<endl;
        vsize+=m.v_size;
        fsize+=m.t_size;
        models.push_back(m);
    }
    cout<<"- Total: "<<vsize<<" vertices, "<<fsize<<" triangles, and "<<input_filenames.size()<<" models"<<endl;
    computeCOM_R();
	
	//read mocap skeleton and animation
	Library::init(mocap_dir);

	//setup binding pose
	setupBindingPose();

    return true;
}

void computeCOM_R()
{
    //compute a bbox
    double box[6]={FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX};
    //-------------------------------------------------------------------------
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            if(p[0]<box[0]) box[0]=p[0];
            if(p[0]>box[1]) box[1]=p[0];
            if(p[1]<box[2]) box[2]=p[1];
            if(p[1]>box[3]) box[3]=p[1];
            if(p[2]<box[4]) box[4]=p[2];
            if(p[2]>box[5]) box[5]=p[2];
        }//j
    }//i

    //-------------------------------------------------------------------------
    // compute center of mass and R...
    COM.set( (box[1]+box[0])/2,(box[3]+box[2])/2,(box[5]+box[4])/2);

    //-------------------------------------------------------------------------
	R=0;
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            float d=(float)(p-COM).normsqr();
            if(d>R) R=d;
        }//j
    }//i

    R=sqrt(R);
}

void setupBindingPose()
{
	//set binding pose to zero pose
	Library::Motion const &mo = Library::motion(0);
	mo.get_pose(0, BindingPose);

	BindingPose.root_position = Vector3d(0, 0, 0);
	BindingPose.root_orientation = Quaternion();
	for (int k = 0; k<BindingPose.bone_orientations.size(); k++) 
		BindingPose.bone_orientations[k] = Quaternion();

	BindingPose.root_position[1] = BindingPose.root_position[1] - .4;
	//right side arms
	BindingPose.bone_orientations[8] = Quaternion::get(raid(5), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[9] = Quaternion::get(raid(-5), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[10] = Quaternion::get(raid(-40), Vector3d(0, 1, 0))*Quaternion::get(raid(-4), Vector3d(0, 0, 1));
	BindingPose.bone_orientations[11] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[12] = Quaternion::get(raid(1), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[13] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[14] = Quaternion::get(raid(-10), Vector3d(0, 1, 0));

	//left side arms
	BindingPose.bone_orientations[18] = Quaternion::get(raid(-5), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[19] = Quaternion::get(raid(5), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[20] = Quaternion::get(raid(35), Vector3d(0, 1, 0))*Quaternion::get(raid(4), Vector3d(0, 0, 1));
	BindingPose.bone_orientations[21] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[22] = Quaternion::get(raid(-1), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[23] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[25] = Quaternion::get(raid(10), Vector3d(0, 1, 0));

	//right leg
	BindingPose.bone_orientations[25] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[26] = Quaternion::get(raid(15), Vector3d(0, 0, 1))*Quaternion::get(raid(-5), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[27] = Quaternion::get(raid(3), Vector3d(0, 0, 1))*Quaternion::get(raid(15), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[28] = Quaternion::get(raid(5), Vector3d(0, 1, 0))*Quaternion::get(raid(-20), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[29] = Quaternion::get(raid(0), Vector3d(0, 1, 0));

	//left leg
	BindingPose.bone_orientations[0] = Quaternion::get(raid(0), Vector3d(0, 1, 0));
	BindingPose.bone_orientations[1] = Quaternion::get(raid(-15), Vector3d(0, 0, 1))*Quaternion::get(raid(-5), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[2] = Quaternion::get(raid(-3), Vector3d(0, 0, 1))*Quaternion::get(raid(15), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[3] = Quaternion::get(raid(5), Vector3d(0, 1, 0))*Quaternion::get(raid(-20), Vector3d(1, 0, 0));
	BindingPose.bone_orientations[4] = Quaternion::get(raid(0), Vector3d(0, 1, 0));

	//
	// TODO: Determine the Binding Pose, you can do it manually 
	//       or you can build a GUI to help you determine the binding pose
	//       The binding pose is a pose that matches the input mesh
	//

	//test start
	//you should remove the following two lines
	//double r = PI / 2;
	//BindingPose.bone_orientations[9] = Quaternion::get(r, Vector3d(0, 1, 0));
	//test end
}

//
// This function will be called when "B" (captial b) is pressed
//
void bind2skin()
{
	if (BindToSkin) return; //already binded

	Character::WorldBones* wb = new Character::WorldBones();
	Character::get_world_bones(BindingPose, *wb);
	//work on the first model only
	model& model = models.front();
	for (unsigned int i = 0; i < model.v_size; i++){
		vertex & temp = model.vertices[i];
		double sum = 0;
		vector<Vector3d> holder;
		vector<double> closeholder;
		Vector3d p = Vector3d(temp.p[0], temp.p[1], temp.p[2]);
		double totaldis = 999;
		int jvalue;
		Vector3d bscinput;
		bool flag = true;
		for (int j = 0; j < 30; j++){
			//Vector3d *extra = new Vector3d(-wb->orientations[j]).rotate(p - wb->bases[j]);
			holder.push_back((-wb->orientations[j]).rotate(p - wb->bases[j]));

			//Calculates the distance from the bone
			Vector3d a = wb->bases[j];
			Vector3d b = wb->tips[j];
			Vector3d ab = (b - a);
			double sqab = ab*ab;
			Vector3d ap = (p - a);
			double t = (ap*ab) / sqab;
			Vector3d sol;
			if (t < 0){
				sol = a;
			}
			else if (t>1){
				sol = b;
			}
			else{
				sol = a + t*ab;
			}
			double  xd = p[0] - sol[0];
			double	yd = p[1] - sol[1];
			double  zd = p[2] - sol[2];
			double dis = sqrt(xd*xd + yd*yd + zd*zd);

			//finds the closest bone
			if (dis < totaldis){
				
				bscinput=(-wb->orientations[j]).rotate(p - wb->bases[j]);
				jvalue = j;
				totaldis = dis;
			}

			//gets close bones 
			if (dis < .3){
				sum += (1 / dis);
				closeholder.push_back(dis);
				flag = false;
			}
			else{
				closeholder.push_back(0);
			}


		}
		//divides the inverse by the sumation
		for (int z = 0; z < closeholder.size(); z++){
			
			
			double weight = (1 / closeholder[z]) / sum;
			
			if (closeholder[z]==0.0){
				closeholder[z] = 0.0;
			}
			else{
				closeholder[z] = weight;
			}
		}
		//if not bone was selected choose the closest
		if (flag){
			holder[jvalue] = bscinput;
			closeholder[jvalue] = 1.0;
		}
		BSC.push_back(bscinput);
		BoneIndex.push_back(jvalue);
		BoneSpaceCoordinates.push_back(holder);
		SkinningWeights.push_back(closeholder);
	
	}
	
	BindToSkin = true;
}

//TODO: skeleton-subspace deformation. perform SSD 
void SSD()
{
	//
	//work on the first model only
	//
	Library::Motion const &mo = Library::motion(currentMotion);
	Character::Pose pos;
	mo.get_pose(currentFrame, pos);

	Character::WorldBones* wb = new Character::WorldBones();
	Character::get_world_bones(pos, *wb);

	model& model = models.front();
	Vector3d sumloc = Vector3d(0, 0, 0);
	double w=0;
	for (unsigned int i = 0; i < model.v_size; i++) {
		Point3d& v = model.vertices[i].p;
		sumloc[0]= 0;
		sumloc[1] = 0;
		sumloc[2] = 0;
		w = 0;
		for (int j = 0; j < 30; j++) { 
			//wiWIB^-1 calculations for all the bones
			w = SkinningWeights[i][j];
			if (w > 0.0) {
				sumloc = sumloc + w * (wb->bases[j] + wb->orientations[j].rotate(BoneSpaceCoordinates[i][j]));
			}
		}
		
		v[0] = sumloc[0];
		v[1] = sumloc[1];
		v[2] = sumloc[2];
		
		
		
	}
	//
	// recompute the position of each vertex in model
	// using BoneSpaceCoordinates and SkinningWeights
	//
}

//-----------------------------------------------------------------------------
//
//
//
//  Open GL stuff below
//
//
//-----------------------------------------------------------------------------

#include <draw.h>




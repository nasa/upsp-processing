#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <sstream>

using namespace std;

int nNodes ( const char * file ) {
  streampos begin,end;
  ifstream myfile (file, ios::binary);
  begin = myfile.tellg();
  myfile.seekg (0, ios::end);
  end = myfile.tellg();
  myfile.close();
  return ( end-begin )/4;
}

// Paul Stremel
// Science and Technology Corp.
// June 23, 2020

// icc xyz_scalar_to_tbl_delta.cpp -o xyz_scalar_to_tbl_delta

// Usage: xyz_scalar_to_tbl_delta grid.p3d X Y Z scalar1 scalar2

// Commadline inputs
//   grid: grid.p3d (little endian, single precision)
//   X: X coordinate flat file
//   Y: Y coordinate flat file
//   Z: Z coordinate flat file
//   scalar1: scalar1 flat file, steady_state or rms
//   scalar2: scalar2 flat file, steady_state or rms

// Reads
//   original grid: grid.p3d
//   X: X coordinate flat file
//   Y: Y coordinate flat file
//   Z: Z coordinate flat file
//   scalar1: scalar1 flat file, steady_state or rms
//   scalar2: scalar2 flat file, steady_state or rms
//   delta: scalar1 - scalar2
// Writes
//   tecplot table: xyz_scalar_delta.tecplot

int main(int argc, char* argv[]){

  if (argc < 7) {
     std::cerr << "Usage: " << argv[0] << " grid.p3d X Y Z scalar1 scalar2" << std::endl;
      return 1;
  }
  string g(argv[1]);
  string x(argv[2]);
  string y(argv[3]);
  string z(argv[4]);
  string s1(argv[5]);
  string s2(argv[6]);

  int gfile=open(g.c_str(), O_RDONLY);
  if(gfile<=0){
    cout<<" Unable to open "<< g <<" for input, Exiting...\n";
    return 1;
  }
  int xfile=open(x.c_str(), O_RDONLY);
  if(xfile<=0){
    cout<<" Unable to open "<< x <<" for input, Exiting...\n";
    return 1;
  }
  int yfile=open(y.c_str(), O_RDONLY);
  if(yfile<=0){
    cout<<" Unable to open "<< y <<" for input, Exiting...\n";
    return 1;
  }
  int zfile=open(z.c_str(), O_RDONLY);
  if(zfile<=0){
    cout<<" Unable to open "<< z <<" for input, Exiting...\n";
    return 1;
  }
  int s1file=open(s1.c_str(), O_RDONLY);
  if(s1file<=0){
    cout<<" Unable to open "<< s1 <<" for input, Exiting...\n";
    return 1;
  }
  int s2file=open(s2.c_str(), O_RDONLY);
  if(s2file<=0){
    cout<<" Unable to open "<< s2 <<" for input, Exiting...\n";
    return 1;
  }

  FILE * ofile=fopen("xyz_scalar_delta.tecplot","w");

  if(ofile==NULL){
    cout<<" Unable to open xyz_scalar_delta.tecplot for output, Exiting...\n";
    return 1;
  }

  fprintf(ofile, "%s", "TITLE = \"Surface Cp\"\n");
  fprintf(ofile, "%s", "VARIABLES = \"x\",\"y\",\"z\",\"Scalar\"\n");

  int var,i;
  float val;
  int ng;
  vector<int> jd;
  vector<int> kd;
  vector<int> ld;

  int nvals;

  // read int
  read(gfile, &var, sizeof(int));

  // read ng
  read(gfile, &var, sizeof(int));
  ng=var;

  // read 2 ints
  read(gfile, &var, sizeof(int));
  read(gfile, &var, sizeof(int));
  for(i=0; i<ng; i++){
    read(gfile, &var, sizeof(int));
    jd.push_back(var);
    read(gfile, &var, sizeof(int));
    kd.push_back(var);
    read(gfile, &var, sizeof(int));
    ld.push_back(var);
  }

  // check total nodes
  int tot_nodes=0;
  for(int ig=0; ig<ng; ig++){
    int n=jd[ig]*kd[ig]*ld[ig];
    tot_nodes=tot_nodes+n;
  }

  int nx=nNodes(x.c_str());
  int ny=nNodes(y.c_str());
  int nz=nNodes(z.c_str());
  int ns1=nNodes(s1.c_str());
  int ns2=nNodes(s2.c_str());
  if(nx != tot_nodes){
    cout << x.c_str()<<" nodes(" << nx << ") != "<< g.c_str() << " nodes("<<tot_nodes<<")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if(ny != tot_nodes){
    cout << y.c_str()<<" nodes(" << ny << ") != "<< g.c_str() << " nodes("<<tot_nodes<<")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if(nz != tot_nodes){
    cout << z.c_str()<<" nodes(" << nz << ") != "<< g.c_str() << " nodes("<<tot_nodes<<")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if(ns1 != tot_nodes){
    cout << s1.c_str()<<" nodes(" << ns1 << ") != "<< g.c_str() << " nodes("<<tot_nodes<<")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if(ns2 != tot_nodes){
    cout << s2.c_str()<<" nodes(" << ns2 << ") != "<< g.c_str() << " nodes("<<tot_nodes<<")\n";
    cout << " Exiting...\n";
    return 1;
  }

  float xval,yval,zval,s1val,s2val,delval;
  for(int ig=0; ig<ng; ig++){
    int igp=ig+1;
    int n=jd[ig]*kd[ig]*ld[ig];
    fprintf(ofile, "%s%i%s%i%s%i%s","ZONE T=\"grid.",igp,"\",F=POINT, I=",jd[ig],", J=",kd[ig],"\n");
    for(i=1; i<=n; i++){
      read(xfile, &xval, sizeof(float));
      read(yfile, &yval, sizeof(float));
      read(zfile, &zval, sizeof(float));
      read(s1file, &s1val, sizeof(float));
      read(s2file, &s2val, sizeof(float));
      if(s1val!=s1val){s1val=0.0;}
      if(s2val!=s2val){s2val=0.0;}
      delval=s1val-s2val;
      fprintf(ofile, "%f %f %f %f %s", xval, yval, zval, delval, "\n");
    }
  }

  return 0;
}

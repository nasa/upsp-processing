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

int nNodes(const char *file)
{
  streampos begin, end;
  ifstream myfile(file, ios::binary);
  begin = myfile.tellg();
  myfile.seekg(0, ios::end);
  end = myfile.tellg();
  myfile.close();
  return (end - begin) / 4;
}

// Paul Stremel
// Science and Technology Corp.
// June 23, 2020

// icc xyz_scalar_to_tbl.cpp -o xyz_scalar_to_tbl

// Usage: xyz_scalar_to_tbl grid.p3d X Y Z scalar

// Commadline inputs
//   grid: grid.p3d (little endian, single precision)
//   X: X coordinate flat file
//   Y: Y coordinate flat file
//   Z: Z coordinate flat file
//   scalar: scalar flat file, steady_state or rms

// Reads
//   grid: grid.p3d
//   X: X coordinate flat file
//   Y: Y coordinate flat file
//   Z: Z coordinate flat file
//   scalar: scalar flat file, steady_state or rms
// Writes
//   tecplot table: xyz_scalar.tecplot

int main(int argc, char *argv[])
{

  if (argc < 7)
  {
    std::cerr << "Usage: " << argv[0] << " [p3d] [X] [Y] [Z] [scalar] [output]" << std::endl;
    return 1;
  }
  string g(argv[1]);
  string x(argv[2]);
  string y(argv[3]);
  string z(argv[4]);
  string s(argv[5]);
  string out(argv[6]);

  int gfile = open(g.c_str(), O_RDONLY);
  if (gfile <= 0)
  {
    cout << " Unable to open " << g << " for input, Exiting...\n";
    return 1;
  }
  int xfile = open(x.c_str(), O_RDONLY);
  if (xfile <= 0)
  {
    cout << " Unable to open " << x << " for input, Exiting...\n";
    return 1;
  }
  int yfile = open(y.c_str(), O_RDONLY);
  if (yfile <= 0)
  {
    cout << " Unable to open " << y << " for input, Exiting...\n";
    return 1;
  }
  int zfile = open(z.c_str(), O_RDONLY);
  if (zfile <= 0)
  {
    cout << " Unable to open " << z << " for input, Exiting...\n";
    return 1;
  }
  int sfile = open(s.c_str(), O_RDONLY);
  if (sfile <= 0)
  {
    cout << " Unable to open " << s << " for input, Exiting...\n";
    return 1;
  }

  FILE *ofile = fopen(out.c_str(), "w");

  if (ofile == NULL)
  {
    cout << " Unable to open xyz_scalar.tecplot for output, Exiting...\n";
    return 1;
  }

  fprintf(ofile, "%s", "TITLE = \"Surface Cp\"\n");
  fprintf(ofile, "%s", "VARIABLES = \"x\",\"y\",\"z\",\"Scalar\"\n");

  int var, i;
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
  ng = var;

  // read 2 ints
  read(gfile, &var, sizeof(int));
  read(gfile, &var, sizeof(int));
  for (i = 0; i < ng; i++)
  {
    read(gfile, &var, sizeof(int));
    jd.push_back(var);
    read(gfile, &var, sizeof(int));
    kd.push_back(var);
    read(gfile, &var, sizeof(int));
    ld.push_back(var);
  }

  // check total nodes
  int tot_nodes = 0;
  for (int ig = 0; ig < ng; ig++)
  {
    int n = jd[ig] * kd[ig] * ld[ig];
    tot_nodes = tot_nodes + n;
  }

  int nx = nNodes(x.c_str());
  int ny = nNodes(y.c_str());
  int nz = nNodes(z.c_str());
  int ns = nNodes(s.c_str());
  if (nx != tot_nodes)
  {
    cout << x.c_str() << " nodes(" << nx << ") != " << g.c_str() << " nodes(" << tot_nodes << ")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if (ny != tot_nodes)
  {
    cout << y.c_str() << " nodes(" << ny << ") != " << g.c_str() << " nodes(" << tot_nodes << ")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if (nz != tot_nodes)
  {
    cout << z.c_str() << " nodes(" << nz << ") != " << g.c_str() << " nodes(" << tot_nodes << ")\n";
    cout << " Exiting...\n";
    return 1;
  }
  else if (ns != tot_nodes)
  {
    cout << s.c_str() << " nodes(" << ns << ") != " << g.c_str() << " nodes(" << tot_nodes << ")\n";
    cout << " Exiting...\n";
    return 1;
  }

  float xval, yval, zval, sval;
  for (int ig = 0; ig < ng; ig++)
  {
    int igp = ig + 1;
    int n = jd[ig] * kd[ig] * ld[ig];
    fprintf(ofile, "%s%i%s%i%s%i%s", "ZONE T=\"grid.", igp, "\",F=POINT, I=", jd[ig], ", J=", kd[ig], "\n");
    for (i = 1; i <= n; i++)
    {
      read(xfile, &xval, sizeof(float));
      read(yfile, &yval, sizeof(float));
      read(zfile, &zval, sizeof(float));
      read(sfile, &sval, sizeof(float));
      if (sval != sval)
      {
        sval = 0.0;
      }
      fprintf(ofile, "%f %f %f %f %s", xval, yval, zval, sval, "\n");
    }
  }

  return 0;
}

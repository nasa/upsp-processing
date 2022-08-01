#ifndef _psp_fio_h_
#define _psp_fio_h_

#include <vector>
#include <unistd.h>   // for read/write
#include <sys/types.h>
#include <sys/stat.h> // for stat()
#include <fcntl.h>    // for open()

#include <regex.h>    // for regexp
#include <dirent.h>   // for readdir

#include "pspSwapit.h"
#include "pspError.h"

namespace fio {

  template <typename T>
    void get(int fin, T* t)
  {
    ssize_t res=read(fin,t,sizeof(T));
    if( res != sizeof(T) ) THROW("read() failed");
  }

  inline void put(int fout, const void* vbuf, size_t bytes)
  {
    const size_t BUFSIZE=1024*1024*100;
    const char* buf = reinterpret_cast<const char*>(vbuf);
    size_t bytesWrote=0,bytesLeft=bytes;
    while( bytesLeft ){
      size_t req = bytesLeft>BUFSIZE ? BUFSIZE : bytesLeft;
      ssize_t res = write(fout,&buf[bytesWrote],req);
      if( res <= 0) ETHROW("write() failed");
      bytesLeft -= res;
      bytesWrote += res;
    }
  }

  template <typename T>
  inline void put(int fout, std::vector<T>& buf)
  {
    size_t words=buf.size();
    size_t bytes=words*sizeof(T);
    put(fout, buf.data(), bytes);
  }

  inline void get(int fin, void* vbuf, size_t bytes)
  {
    char* buf = reinterpret_cast<char*>(vbuf);
    size_t bytesRead=0,bytesLeft=bytes;
    while( bytesLeft ){
      size_t req = bytesLeft;
      ssize_t res = read(fin,&buf[bytesRead],req);
      if( res <= 0) {
	std::cerr<<"      req = "<<req<<"\n";
	std::cerr<<"      res = "<<res<<"\n";
	std::cerr<<"bytesLeft = "<<bytesLeft<<"\n";
	std::cerr<<"bytesRead = "<<bytesRead<<"\n";
	std::cerr<<"  filepos = "<<lseek(fin,0,SEEK_CUR)<<"\n";
	ETHROW("read() failed");
      }
      bytesLeft -= res;
      bytesRead += res;
    }
  }

  inline void get(int fin, void* vbuf, size_t bytes, off_t pos)
  {
    char* buf = reinterpret_cast<char*>(vbuf);
    size_t bytesRead=0,bytesLeft=bytes;
    off_t got = lseek(fin,pos,SEEK_SET);
    if( got != pos ) {
	std::cerr<<"      req = "<<pos<<"\n";
	std::cerr<<"      res = "<<got<<"\n";
	ETHROW("lseek() failed");
    }
    while( bytesLeft ){
      size_t req = bytesLeft;
      ssize_t res = read(fin,&buf[bytesRead],req);
      if( res <= 0) {
	std::cerr<<"      req = "<<req<<"\n";
	std::cerr<<"      res = "<<res<<"\n";
	std::cerr<<"bytesLeft = "<<bytesLeft<<"\n";
	std::cerr<<"bytesRead = "<<bytesRead<<"\n";
	std::cerr<<"  filepos = "<<lseek(fin,0,SEEK_CUR)<<"\n";
	ETHROW("read() failed");
      }
      bytesLeft -= res;
      bytesRead += res;
    }
  }

  template <typename T>
  inline void get(int fin, std::vector<T>& buf)
  {
    size_t words=buf.size();
    size_t bytes=words*sizeof(T);
    get(fin, (char*)&buf[0], bytes);
  }

  inline void pget(int fin, void* buf, size_t bytes, off_t pos)
  {
    char* dst = reinterpret_cast<char*>(buf);
    const size_t BUFSIZE=1024*1024*1000;
    size_t bytesRead=0,bytesLeft=bytes;
    off_t curpos=pos;
    while( bytesLeft ){
      size_t req = bytesLeft>BUFSIZE ? BUFSIZE : bytesLeft;
      ssize_t res = pread(fin,&dst[bytesRead],req,curpos);
      if( res <= 0) {
	std::cerr<<"      req = "<<req<<"\n";
	std::cerr<<"      res = "<<res<<"\n";
	std::cerr<<"bytesLeft = "<<bytesLeft<<"\n";
	std::cerr<<"bytesRead = "<<bytesRead<<"\n";
	std::cerr<<"  filepos = "<<lseek(fin,0,SEEK_CUR)<<"\n";
	ETHROW("read() failed");
      }
      bytesLeft -= res;
      bytesRead += res;
      curpos += res;
    }
  }

  inline void pput(int fout, void* buf, size_t bytes, off_t pos)
  {
    char* src = reinterpret_cast<char*>(buf);
    const size_t BUFSIZE=1024*1024*1000;
    size_t bytesWrote=0,bytesLeft=bytes;
    off_t curpos=pos;
    while( bytesLeft ){
      size_t req = bytesLeft>BUFSIZE ? BUFSIZE : bytesLeft;
      ssize_t res = pwrite(fout,&src[bytesWrote],req,curpos);
      if( res <= 0) {
	std::cerr<<"       req = "<<req<<"\n";
	std::cerr<<"       res = "<<res<<"\n";
	std::cerr<<" bytesLeft = "<<bytesLeft<<"\n";
	std::cerr<<"bytesWrote = "<<bytesWrote<<"\n";
	std::cerr<<"   filepos = "<<lseek(fout,0,SEEK_CUR)<<"\n";
	ETHROW("read() failed");
      }
      bytesLeft -= res;
      bytesWrote += res;
      curpos += res;
    }
  }

  template <typename T>
  inline void load(std::string fname, std::vector<T>& buf)
  {
    struct stat statbuf;
    if( stat(fname.c_str(), &statbuf) )
      THROW("stat() failed on ["+fname+"]");
    size_t filesize = statbuf.st_size;
    size_t nelem = filesize/sizeof(T);
    buf.resize(nelem);
    int fd=open(fname.c_str(),O_RDONLY);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    char* bufptr=(char*)&buf[0];
    get(fd,bufptr,filesize);
    close(fd);
  }

  template <typename T>
    inline void load(std::string fname, std::vector<T>& buf, size_t userbytes)
  {
    struct stat statbuf;
    if( stat(fname.c_str(), &statbuf) )
      THROW("stat() failed on ["+fname+"]");
    size_t filesize = statbuf.st_size;
    size_t readsize = std::min(filesize,userbytes);
    size_t nelem = readsize/sizeof(T);
    buf.resize(nelem);
    int fd=open(fname.c_str(),O_RDONLY);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    char* bufptr=(char*)&buf[0];
    get(fd,bufptr,readsize);
    close(fd);
  }
  template <typename T>
    inline void load(std::string fname, std::vector<T>& buf, off_t offset, size_t userbytes)
  {
    struct stat statbuf;
    if( stat(fname.c_str(), &statbuf) )
      THROW("stat() failed on ["+fname+"]");
    size_t filesize = statbuf.st_size;
    size_t readsize = std::min(filesize,userbytes);
    size_t nelem = readsize/sizeof(T);
    buf.resize(nelem);
    int fd=open(fname.c_str(),O_RDONLY);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    off_t got = lseek(fd,offset,SEEK_SET);
    if( got != offset ) THROW("lseek() error: ["+fname+"]");
    char* bufptr=(char*)&buf[0];
    get(fd,bufptr,readsize);
    close(fd);
  }

  template <typename T>
  inline void add(std::string fname, std::vector<T>& buf)
  {
    struct stat statbuf;
    if( stat(fname.c_str(), &statbuf) )
      THROW("stat() failed on ["+fname+"]");
    size_t filesize = statbuf.st_size;
    size_t nelem = filesize/sizeof(T);
    size_t cursize = buf.size();
    buf.resize(cursize+nelem);
    int fd=open(fname.c_str(),O_RDONLY);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    char* bufptr=(char*)&buf[cursize];
    get(fd,bufptr,filesize);
    close(fd);
  }

  template <typename T>
  inline void dump(std::string fname, std::vector<T>& buf)
  {
    size_t bytes=buf.size()*sizeof(T);
    int fd=open(fname.c_str(),O_WRONLY|O_TRUNC|O_CREAT,0644);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    put(fd,buf.data(),bytes);
    close(fd);
  }

  inline void dump(std::string fname, const void* buf, size_t bytes)
  {
    int fd=open(fname.c_str(),O_WRONLY|O_TRUNC|O_CREAT,0644);
    if( fd==-1 ) THROW("open() failed on ["+fname+"]");
    put(fd,buf,bytes);
    close(fd);
  }

  inline bool fileHasExt(std::string file, std::string e)
  {
    std::string::size_type ppos = file.find_last_of(".");
    if( ppos==std::string::npos ) return false;
    std::string ext(file,ppos+1);
    return not e.compare(ext);
  }

  inline bool fileExists(std::string fname, size_t* bytes=0)
  {
    struct stat statbuf;
    int res=stat(fname.c_str(), &statbuf);
    if( bytes ) *bytes = statbuf.st_size;
    return res==0;
  }

  inline bool dirExists(std::string dirname)
  {
    struct stat statbuf;
    int res=stat(dirname.c_str(), &statbuf);
    if( res != 0 ) return false;
    return statbuf.st_mode & S_IFDIR;
  }

#if 0
  inline void getfiles(std::string dir, std::string pat, std::vector<std::string>& fnames)
  {
    struct stat sbuf;
    int res = stat(dir.c_str(),&sbuf);
    if( res != 0 ){
      ERROR("cannot read dir ["+std::string(dir)+"]");
      return;
    }

    regex_t regptr;
    if (regcomp(&regptr, pat.c_str(), REG_EXTENDED|REG_ICASE) != 0){
      ERROR("Error compiling regular expression");
      return;
    }
    DIR *dirobj = opendir(dir.c_str());
    if( dirobj == 0 ){
      ERROR("opendir() failed for ["+std::string(dir)+"]");
      return;
    }
    struct dirent *entry = readdir(dirobj);
    while( entry ){
      if( strcmp(".",entry -> d_name) && strcmp("..",entry -> d_name) ){
	const size_t NMATCH=5;
	regmatch_t pmatch[NMATCH];
	for(unsigned i=0;i<NMATCH;++i){
	  pmatch[i].rm_so=pmatch[i].rm_eo=0;
	}
	int status = regexec(&regptr,entry -> d_name,NMATCH,pmatch,0);
	if( status==0 ){
	  fnames.push_back(dir + "/" + entry -> d_name);
	}
      }
      entry = readdir(dirobj);
    }
    closedir(dirobj);
  }
#endif

  /*
  inline unsigned short getbareshort(int fin, bool needswap=false)
  {
    unsigned short word;
    ssize_t res=read(fin,&word,sizeof(unsigned short));
    if( res<=0 )THROW("getbareword() failed");
    if( needswap ) word = bin::swap((u_int16_t)word);
    return word;
  }

  inline unsigned getbareword(int fin, bool needswap=false)
  {
    unsigned word;
    ssize_t res=read(fin,&word,sizeof(unsigned));
    if( res<=0 )THROW("getbareword() failed");
    if( needswap ) word = bin::swap((u_int32_t)word);
    return word;
  }

  inline int getbareint(int fin, bool needswap=false)
  {
    int word;
    ssize_t res=read(fin,&word,sizeof(int));
    if( res<=0 )THROW("getbareword() failed");
    if( needswap ) word = bin::swap32(&word,1);
    return word;
  }

  inline void getbareints(int fin, size_t n, int* results, bool needswap=false)
  {
    ssize_t res=read(fin,results,n*sizeof(int));
    if( res<=0 )THROW("getbareints() failed");
    if( needswap ) bin::swap32((unsigned*)results,n);
  }

  inline void getbarefloats(int fin, size_t n, float* results, bool needswap=false)
  {
    ssize_t res=read(fin,results,n*sizeof(float));
    if( res<=0 )THROW("getbarefloats() failed");
    if( needswap ) bin::swap32((u_int32_t*)results,n);
  }

  inline void getbaredoubles(int fin, size_t n, double* results, bool needswap=false)
  {
    ssize_t res=read(fin,results,n*sizeof(double));
    if( res<=0 )THROW("getbaredoubles() failed");
    if( needswap ) bin::swap64((u_int64_t*)results,n);
  }

  inline float getbarefloat(int fin, bool needswap=false)
  {
    float word;
    ssize_t res=read(fin,&word,sizeof(float));
    if( res<=0 ) THROW("getbareword() failed");
    if( needswap ) word = bin::swap(word);
    return word;
  }

  inline double getbaredouble(int fin, bool needswap=false)
  {
    double word;
    ssize_t res=read(fin,&word,sizeof(double));
    if( res<=0 ) THROW("getbareword() failed");
    if( needswap ) word = bin::swap(word);
    return word;
  }
  */

}

#endif

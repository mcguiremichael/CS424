#include <iostream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using namespace std;
void getdir (string dir)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return ;
    }

    while ((dirp = readdir(dp)) != NULL) {
        string file_name(string(dirp->d_name));
        string end = ".jpg";
        if(file_name.find(end) != string::npos){
        cout<<dir+file_name<<endl;
    }
       //files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return ;
}

int main(){
    cout<<"reading files from the directory "<< "object_identification/query-image/"<<endl;
    getdir("object_identification/query-image/");
    return 0;


}

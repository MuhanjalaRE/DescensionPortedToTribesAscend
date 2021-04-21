#pragma once
#include <windows.h>
#include <string>
#include <vector>

using namespace std;

namespace UsefulSnippets {
namespace Files {

class FileObject {
   private:
    string path;
    string file_name;
    string absolute_path;
    string file_path;

   public:
    FileObject(string path, string file_name);
    void printInfo(void);
    string getAbsolutePath(void);
    string getFileName(void);
    const char* getFileName_cstr(void);
    const char* getFilePath_cstr(void);
};

vector<FileObject> getFiles(const char* path = "", const char* file_string = "", bool use_cwd = true);

vector<string> getFolders(const char* path = "", const char* folder_string = "", bool use_cwd = true);

string getPathString(const char* path = "", const char* file_string = "", bool use_cwd = true);

}  // namespace Files
}  // namespace UsefulSnippets
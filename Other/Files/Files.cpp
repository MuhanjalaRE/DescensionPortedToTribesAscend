#include "Files.h"
#include <algorithm>
#include <iostream>

using namespace std;

namespace UsefulSnippets {
namespace Files {

string FileObject::getAbsolutePath(void) {
    return this->absolute_path;
}

string FileObject::getFileName(void) {
    return this->file_name;
}

const char* FileObject::getFileName_cstr(void) {
    return this->file_name.c_str();
}

const char* FileObject::getFilePath_cstr(void) {
    return this->file_path.c_str();
}

FileObject::FileObject(string path, string file_name) {
    this->path = path;
    this->file_name = file_name;

    char cs_file_path[256];
    strcpy(cs_file_path, path.c_str());
    strcat(cs_file_path, file_name.c_str());

    this->file_path = std::string(cs_file_path);

    char cs_absolute_path_buffer[256];
    GetFullPathNameA(cs_file_path, 256, cs_absolute_path_buffer, NULL);

    // cout << path << endl << file_name << endl;

    // cout << cs_absolute_path_buffer << endl;

    this->absolute_path = string(cs_absolute_path_buffer);
}

void FileObject::printInfo(void) {
    wcout << "FileObject:" << endl;
    wcout << "\tFile path: " << path.c_str() << endl;
    wcout << "\tFile name: " << file_name.c_str() << endl;
    wcout << "\tAbsolute path: " << absolute_path.c_str() << endl;
}

string getPathString(const char* path, const char* file_string, bool use_cwd) {
    char path_[256];
    char dir[256];
    strcpy(path_, "");

    if (use_cwd) {
        strcpy(path_, ".\\");
    }
    strcat(path_, path);
    if (strcmp(path, "") == 0) {
        // strcat(path_, ".\\");
    }
    strcat(path_, "\\");
    strcpy(dir, path_);

    FileObject o_file_object = FileObject(dir, file_string);

    return string(o_file_object.getFilePath_cstr());
}

vector<FileObject> UsefulSnippets::Files::getFiles(const char* path, const char* file_string, bool use_cwd) {
    /*
    Path stuff:
    "." means cwd
    end all directory/path with "*"
    */

    char path_[256];
    char dir[256];
    strcpy(path_, "");

    if (use_cwd) {
        strcpy(path_, ".\\");
    }
    strcat(path_, path);
    if (strcmp(path, "") == 0) {
        // strcat(path_, ".\\");
    }
    strcat(path_, "\\");
    strcpy(dir, path_);
    strcat(path_, "*");

    WIN32_FIND_DATA FindFileData;
    HANDLE hFind;
    hFind = FindFirstFile(path_, &FindFileData);

    // while (hFind != INVALID_HANDLE_VALUE) {
    //  cout << FindFileData.cFileName << endl;
    //  FindNextFile(hFind, &FindFileData);
    //}

    vector<FileObject> v_files_vector;

    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            if (!(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                string s_file_name(FindFileData.cFileName);
                char cs_file_name[100];
                transform(s_file_name.begin(), s_file_name.end(), s_file_name.begin(), tolower);
                strcpy(cs_file_name, s_file_name.c_str());
                if (strstr(cs_file_name, file_string) != NULL) {
                    string s_file_name = string(FindFileData.cFileName);
                    // cout << FindFileData.cFileName << endl;
                    FileObject o_file_object = FileObject(dir, s_file_name);
                    v_files_vector.push_back(o_file_object);
                }
            }

        } while (FindNextFile(hFind, &FindFileData) != 0);
    }

    FindClose(hFind);
    return v_files_vector;
}

vector<string> UsefulSnippets::Files::getFolders(const char* path, const char* folder_string, bool use_cwd) {
    /*
    Path stuff:
    "." means cwd
    end all directory/path with "*"
    */

    char path_[256];
    strcpy(path_, "");

    if (use_cwd) {
        strcpy(path_, ".\\");
    }
    strcat(path_, path);
    if (strcmp(path, "") == 0) {
        // strcat(path_, ".\\");
    }
    strcat(path_, "\\*");

    WIN32_FIND_DATA FindFileData;
    HANDLE hFind;
    hFind = FindFirstFile(path_, &FindFileData);

    // while (hFind != INVALID_HANDLE_VALUE) {
    //  cout << FindFileData.cFileName << endl;
    //  FindNextFile(hFind, &FindFileData);
    //}

    vector<string> v_folders_vector;

    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            if ((FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                string s_file_name(FindFileData.cFileName);
                char cs_file_name[100];
                transform(s_file_name.begin(), s_file_name.end(), s_file_name.begin(), tolower);
                strcpy(cs_file_name, s_file_name.c_str());
                if (strstr(cs_file_name, folder_string) != NULL) {
                    // cout << FindFileData.cFileName << endl;
                    v_folders_vector.push_back(string(FindFileData.cFileName));
                }
            }

        } while (FindNextFile(hFind, &FindFileData) != 0);
    }

    FindClose(hFind);
    return v_folders_vector;
}

}  // namespace Files
}  // namespace UsefulSnippets

#include <fstream>
#include <iostream>
std::ifstream infile("container_wipe.pwipe");

void skip_headers(std::ifstream &infile){
    int header_length = 13;
    std::string header_temp;
    for (int i = 0; i < header_length; i++){
        infile >> header_temp;
    }
};

int main(int argc, char** argv){
    std::string line;
    std::ifstream infile("/home/ashis/container_wipe.pwipe");
    std::cout << "Test read.\n" ;
    float j1, j2, j3, j4, j5, j6, x_pos, y_pos, z_pos, qx, qy, qz, qw;

    if (infile.is_open())
    {
        skip_headers(infile);
        while (infile >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x_pos >> y_pos >> z_pos >> qx >> qy >> qz >> qw)
        {
            std::cout << "Reading line.\n";
            std::cout << j1<<j2<<j3<<j4<<j5<<j6<<x_pos<<y_pos<<z_pos<<qx<<qy << qz << qw << std::endl;
        }

    }
    else std::cout << "Unable to open file\n";
    return 0;
}
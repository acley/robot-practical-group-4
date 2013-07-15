#include <iostream>
#include <fstream>
#include <sstream>

int main()
{
  std::ofstream myfile;
  myfile.open ("./config/planning/grid.txt");
  
  int number_of_columns = 5;
  int number_of_rows = 15;
  
  for (int r=1; r<=number_of_rows; r++) {
    for (int c=1; c<=number_of_columns; c++) {
      std::stringstream cell;
      cell << "- \"pos-" << r << "-" << c << " ";
      
      // north
      if (r>1) {
        std::stringstream north;
        north << "pos-" << (r-1) << "-" << c << " dir-north\"";
        myfile << cell.str() << north.str() << std::endl;
      }
      
      // south
      if (r<number_of_rows) {
        std::stringstream south;
        south << "pos-" << (r+1) << "-" << c << " dir-south\"";
        myfile << cell.str() << south.str() << std::endl;
      }
      
      // west
      if (c>1) {
        std::stringstream west;
        west << "pos-" << r << "-" << (c-1) << " dir-west\"";
        myfile << cell.str() << west.str() << std::endl;
      }
      
      // east
      if (c<number_of_columns) {
        std::stringstream east;
        east << "pos-" << r << "-" << (c+1) << " dir-east\"";
        myfile << cell.str() << east.str() << std::endl;
      }
    }
  }
  myfile.close();
}

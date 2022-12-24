#include <utility> // cin getUserInput
#include <limits> // cin getUserInput
#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    // ate at the end
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    // vector of bytes
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    // move transfers contents without pointers or refs
    return std::move(contents);
}

// function to support modularity of user input prompts
// keep asking until user input in valid boundary of 0-100 [%]
float getUserInput(std::string userInputString){
    float userInputValue=-1; // initialize invalid value to ensure user input
    bool keepTrying;
    do{
        keepTrying = false; // assume valid user input
        std::cout << "Enter the value of " << userInputString << " :\n";
        std::cin >> userInputValue;

        // if cin is of invalid type or out of range, keep trying
        if (!std::cin || (userInputValue < 0.0) || (userInputValue > 100.0)){ 
            keepTrying = true; // ask user for other input

            std::cout << "Invalid input, please enter a number between 0 and 100 \n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // ignore rest of current line, up to delimiter '\n'
            // `numeric_limits<streamsize>::max()`sets maximum number of chars to ignore.
            // with upper stream size limit, there is no limit to the number of characters cin is to ignore.            
        } 
    } while(keepTrying); // loop until valid input received
    return userInputValue;
}


int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            // optionally define different .osm data file
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "\n" << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        // osm_data_file = "../map.osm";
        // user defined open street map
        // https://www.openstreetmap.org/export#map=17/47.37601/8.54039
        osm_data_file = "../zh.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    // // testing without user input
    // float start_x=10.0;
    // float start_y=10.0;
    // float end_x=90.0;
    // float end_y=90.0;

    // user input prompt
    std::cout << "\nEnter {x,y} values of path start and end. \n";
    std::cout << "Values are given between 0 and 100 [%] of the OpenStreetMap \n";
    std::cout << "{0,0} is {bottom,left} and {100,100} is {top,right} \n\n";

    // user input get 
    float start_x,start_y,end_x,end_y;
    start_x = getUserInput("start_x");
    start_y = getUserInput("start_y");
    end_x = getUserInput("end_x");
    end_y = getUserInput("end_y");

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x,start_y,end_x,end_y};
    // RoutePlanner route_planner{model, 10, 10, 90, 90};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}

#include <fstream>
#include <iostream>
#include <catch2/catch_test_macros.hpp>

#include "agent/map.h"

TEST_CASE( "Common map usage", "[map]" )
{
    using namespace agent;

    SECTION( "Add valid cells to map" )
    {
        PerceivedMap map{};
        REQUIRE_FALSE( map.isComplete() );

        for(int x =-24; x <= 24; x+=2)
        {
            for(int y =-10; y <= 10; y+=2)
            {
                int id = computeCellId(x,y);
                REQUIRE(map.addCell(id));
                REQUIRE_FALSE(map.cellIsExpanded(id));
                REQUIRE_FALSE(map.isComplete());
                map.setCellExpanded(id, true);
            }
        }

        REQUIRE( map.isComplete() );
    }

    SECTION("Connect neighbors")
    {
        PerceivedMap map{};
        REQUIRE_FALSE( map.isComplete() );

        for(int x =-24; x <= 24; x+=2)
        {
            for(int y =-10; y <= 10; y+=2)
            {
                int id = computeCellId(x,y);
                REQUIRE(map.addCell(id));
            }
        }

        for(int x =-24; x <= 24; x+=2)
        {
            for(int y =-10; y <= 10; y+=2)
            {
                int id1 = computeCellId(x,y);

                for(int dx = -2; dx <= 2; dx+=2)
                {
                    for(int dy = -2; dy <= 2; dy+=2)
                    {
                        if(dx == 0 && dy == 0) continue;
                        int id2 = computeCellId(x+dx,y+dy);
                        if(validateCellId(id2))
                        {
                            // NOTE: Because we are only define neighbors and 
                            //       not expanding cells, each cell will be expanded
                            //       when achieving the maximum number of neighbors
                            map.linkNeighbor(id1, id2);
                            map.linkNeighbor(id2,id1);
                        }
                    }
                }
            }
        }

        REQUIRE( map.isComplete() );
    }
}

TEST_CASE("Invalid cells", "[map]")
{
    using namespace agent;

    SECTION( "Add invalid cells to map" )
    {
        PerceivedMap map{};

        for(int x =-23; x <= 23; x+=2)
        {
            for(int y =-9; y <= 9; y+=2)
            {
                int id = computeCellId(x,y);
                REQUIRE_THROWS_AS( map.addCell(id), std::invalid_argument );
                REQUIRE_THROWS_AS( map.setCellExpanded(id, true), std::invalid_argument );
                REQUIRE_THROWS_AS( map.cellIsExpanded(id), std::invalid_argument );
            }
        }

        REQUIRE_FALSE( map.isComplete() );
    }
}

TEST_CASE( "Small real world scenario", "[map]" )
{
    using namespace agent;

    PerceivedMap map{};
    
    REQUIRE( map.addCell( computeCellId(0,0) ) ); // 514
    REQUIRE( map.addCell( computeCellId(2,0) ) ); // 516
    REQUIRE( map.addCell( computeCellId(0,2) ) ); // 612
    REQUIRE( map.addCell( computeCellId(2,2) ) ); // 614
    
    bool neighbors = map.linkNeighbor(514, 516);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(516, 514);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(514, 612);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(612, 514);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(516, 614);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(614, 516);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(612, 614);
    REQUIRE( neighbors );
    neighbors = map.linkNeighbor(614, 612);
    REQUIRE( neighbors );

    REQUIRE_FALSE( map.isComplete() );
    
    map.setCellExpanded(514 ,true);
    map.setCellExpanded(516, true);
    map.setCellExpanded(612, true);
    map.setCellExpanded(614, true);
    
    map.cellIsExpanded(514);
    map.cellIsExpanded(516);
    map.cellIsExpanded(612);
    map.cellIsExpanded(614);
    
    REQUIRE( map.isComplete() );

    SECTION( "Write map and path to files" )
    {
        std::vector<int> path{514,614};
        map.writeToFile("test", path);
        std::cout << "Done" << std::endl;
    }

    SECTION( "Read map file")
    {
        std::ifstream file;
        file.open("test.map");
        REQUIRE( file.is_open() );
        std::string text{};
        while(!file.eof())
        {
            std::string line;
            std::getline(file, line);
            text += line + '\n';
        }
        file.close();
        text.pop_back(); // remove last \n

        char map[21][49];
        for(int i = 0; i < 21; i++)
        {
            for(int j = 0; j < 49; j++)
            {
                map[i][j] = ' ';
            }
        }

        map[10][24] = '0';
        map[10][25] = '-';
        map[10][26] = ' ';
        map[9][26] = '|';
        map[8][26] = '1';
        map[8][25] = '-';
        map[8][24] = ' ';
        map[9][24] = '|';

        std::string textmap{};
        for(int i = 0; i < 21; i++)
        {
            for(int j = 0; j < 49; j++)
            {
                textmap += map[i][j];
            }
            textmap += '\n';
        }
        textmap.pop_back(); // remove last \n

        REQUIRE( text == textmap );
    }

    SECTION( "Read path file" )
    {
        std::ifstream file;
        file.open("test.path");
        REQUIRE( file.is_open() );
        std::string text{};
        while(!file.eof())
        {
            std::string line;
            std::getline(file, line);
            text += line + '\n';
        }
        file.close();
        text.pop_back(); // remove last \n

        std::string textmap = "0 0\n2 0\n2 2\n2 0\n0 0";

        REQUIRE( text == textmap );
    }
}


// TODO: test getNextCell
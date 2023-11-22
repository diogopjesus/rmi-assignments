// 01-TestPerceivedMap.cpp

#include <fstream>

#include <catch2/catch_test_macros.hpp>

#include "agent/localmap.h"

TEST_CASE( "add cells", "[map]" )
{
    using namespace agent;

    SECTION( "map starting incomplete" )
    {
        LocalMap map{};
        REQUIRE_FALSE( map.isComplete() );
    }
    
    SECTION( "adding valid cells to map" )
    {
        LocalMap map{};

        for(int x =-10; x <= 10; x+=2)
        {
            for(int y =-24; y <= 24; y+=2)
            {
                int id = (y + 24) * 21 + x + 10;
                REQUIRE( map.addCell(x,y) == id );
                REQUIRE( map.cellInPerceivedMap(id) );
                REQUIRE_FALSE( map.cellIsExpanded(id) );
                map.expandCell(id);
                REQUIRE( map.cellIsExpanded(id) );
            }
        }

        REQUIRE( map.isComplete() );
    }

    SECTION( "adding invalid cells to map" )
    {
        LocalMap map{};

        for(int x =-10; x <= 10; x+=2)
        {
            for(int y =-24; y <= 24; y+=2)
            {
                int id = (y + 24) * 21 + x + 10;
                REQUIRE_THROWS_AS( map.addCell(x,y) == id, std::invalid_argument );
                REQUIRE_FALSE( map.cellInPerceivedMap(id) );
                REQUIRE_THROWS_AS( map.cellIsExpanded(id), std::invalid_argument );
                REQUIRE_THROWS_AS( map.expandCell(id), std::invalid_argument );
                REQUIRE_THROWS_AS( map.cellIsExpanded(id), std::invalid_argument );
            }
        }

        REQUIRE_FALSE( map.isComplete() );
    }
}

TEST_CASE( "add cell neighbors", "[map]" )
{
    using namespace agent;

    SECTION( "adding valid neighbors to map" )
    {
        LocalMap map{};

        for(int x =-10; x <= 10; x+=2)
        {
            for(int y =-24; y <= 24; y+=2)
            {
                int id = (y + 24) * 21 + x + 10;
                REQUIRE( map.addCell(x,y) == id );
                REQUIRE( map.cellInPerceivedMap(id) );
                REQUIRE_FALSE( map.cellIsExpanded(id) );
            }
        }

        for(int x =-10; x <= 10; x+=2)
        {
            for(int y =-24; y <= 24; y+=2)
            {
                int id = (y + 24) * 21 + x + 10;
                
                for(int dx = -2; dx <= 2; dx+=2)
                {
                    for(int dy = -2; dy <= 2; dy+=2)
                    {
                        if(dx == 0 && dy == 0) continue;
                        int nei = (y + dy + 24) * 21 + x + dx + 10;
                        REQUIRE( map.addCellNeighbor(id, nei) );
                        REQUIRE( map.cellInPerceivedMap(nei) );
                        if (dx == 2 && dy == 2) REQUIRE( map.cellIsExpanded(nei) );
                        else REQUIRE_FALSE( map.cellIsExpanded(nei) );
                    }
                }
            }
        }

        REQUIRE( map.isComplete() );
    }    
}

TEST_CASE( "map creation", "[map]" )
{
    using namespace agent;

    LocalMap map{};
    
    SECTION( "adding simple real scenario to map" )
    {
        REQUIRE( map.addCell(0,0) == 514 );
        REQUIRE( map.addCell(2,0) == 516 );
        REQUIRE( map.addCell(0,2) == 556 );
        REQUIRE( map.addCell(2,2) == 558 );
        
        REQUIRE( map.addCellNeighbor(514, 516) );
        REQUIRE( map.addCellNeighbor(514, 556) );
        REQUIRE( map.addCellNeighbor(516, 558) );
        REQUIRE( map.addCellNeighbor(556, 558) );

        REQUIRE_FALSE( map.isComplete() );
        
        map.expandCell(514);
        map.expandCell(516);
        map.expandCell(556);
        map.expandCell(558);
        
        map.cellIsExpanded(514);
        map.cellIsExpanded(516);
        map.cellIsExpanded(556);
        map.cellIsExpanded(558);
        
        REQUIRE( map.isComplete() );
    }

    SECTION( "writing map to files" )
    {
        map.writeToFile("test", {514,558});

        SECTION( "Reading map file")
        {
            std::ifstream file;
            file.open("test.map");
            REQUIRE( file.is_open() );
            std::string text{};
            while(!file.eof())
            {
                std::string line;
                std::getline(file, line);
                text += line;
            }
            file.close();

            char map[21][49];
            for(int i = 0; i < 21; i++)
            {
                for(int j = 0; j < 49; j++)
                {
                    map[i][j] = ' ';
                }
            }

            map[10][24] = '0';
            map[11][24] = '-';
            map[12][24] = ' ';
            map[12][23] = '|';
            map[12][22] = '1';
            map[11][22] = '-';
            map[10][22] = ' ';
            map[10][23] = '|';

            std::string textmap{};
            for(int i = 0; i < 21; i++)
            {
                text += std::string(map[i]) + '\n';
            }

            REQUIRE( text == textmap );
        }

        SECTION( "reading path file" )
        {
            std::ifstream file;
            file.open("test.map");
            REQUIRE( file.is_open() );
            std::string text{};
            while(!file.eof())
            {
                std::string line;
                std::getline(file, line);
                text += line;
            }
            file.close();

            std::string textmap = "0 0\n0 2\n2 2\n2 0\n0 0\n";

            REQUIRE( text == textmap );
        }
    }

}


// TODO: test getNextCell
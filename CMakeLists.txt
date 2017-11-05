link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport
		yaml-cpp
		TensegrityModel
    libquadprog.a
    armadillo
    lapack
    blas
    )

  add_executable(TModelTester
    #threeBarModel.cpp
    LengthControllerYAML.cpp
    YAMLSimulator.cpp
) 

  add_executable(DropTheCube
    BallDropperYAML.cpp
    DropTheCube.cpp
) 

  add_executable(CubeSim
    Cubetroller.cpp
    CubeSim.cpp
) 

# UN-COMMENT these lines for 6.1
#add_executable(App3BarYAML
#    LengthControllerYAML.cpp
#    App3BarYAML.cpp
#) 

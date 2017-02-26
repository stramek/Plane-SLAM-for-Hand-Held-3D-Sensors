//
//  main.cpp
//  Projekt Magisterski
//
//  Created by Marcin Stramowski on 15.12.2016.
//  Copyright © 2016 Marcin Stramowski. All rights reserved.
//

#include "../../include/dataset/main.h"

int visualizerTest(int argc, char** argv){
    QApplication application(argc,argv);
    //setlocale(LC_NUMERIC,"C");
    glutInit(&argc, argv);

    QGLVisualizer visu;

    visu.setWindowTitle("Simulator viewer");
    visu.show();

    return application.exec();
}

int main(int argc, char** argv) {
    cout << PHCP_MODEL;
    //visualizerTest(argc, argv);
}

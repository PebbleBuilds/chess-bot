/*

Chess Bot Core
By Rocco Ruan

This is a ROS publisher/subscriber that will serve as the core of my chess robot. It will
interface with the computer vision and chess engine parts of the code via ROS, as well as
the IK code on the arm's Arduino Nano via serial.

It would be so much easier to write this in Python, but I'm forcing myself to write it in
C++ as an educational exercise.

*/

#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;

class ChessBotCore{
    // Access modifier
    private:

    // Data members
    

    // Member Functions

};

int main(){
    int user_input;
    bool human_turn;

    cout << "Thanks for playing with chess-bot!" << endl;
    cout << "What colour would you like to play as? Type 0 for white, and 1 for black." << endl;

    cin >> user_input;

    while(1){
        if(cin.fail()){
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(),'\n');
            cout << "Enter an integer." << endl;

            cin >> user_input;
        }
        else(){
            break;
        }
    }

    if(colour){
        human_turn = true; //White goes first.
        // Tell chess_node.py that the player chose white.

    }
    else{
        human_turn = false;
        // Tell chess_node.py that the player chose black.
    }

    if(human_turn){
        // Wait until player tells us that they have made their move.
        cout << "It's your turn! Press Enter when you've made your move." << endl;
        cin >> user_input;

        // Tell CV to check the position of the pieces, and tell us what it is.

        // Tell the chess engine interface the position of the pieces, and
    }
};
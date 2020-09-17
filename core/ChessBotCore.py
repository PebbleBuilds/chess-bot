# ChessBotCore
# By Rocco Ruan
# 
# A class that runs the core functionality of a chess-playing robot arm, including interfacing with an arm controller object,
# a computer vision object, and a chess engine object powered by Stockfish. This class is in particular responsible for the
# "state-machine"-like behaviour of the chess robot arm, as well as the user interface.


from StockfishInterface import StockfishInterface
from string import ascii_lowercase
import CV_Interface
from ArmController import ArmController

class ChessBotCore():
    def __init__(self, human_side="white"):
        self.human_side = human_side
        self.computer_side = "white" if self.human_side == "black" else "white"
        self.is_human_turn = True if self.human_side == "white" else False
        self.stockfish = StockfishInterface()
        self.stockfish.new_game()
        self.ArmController = ArmController.ArmController()

        #generate lists of occupied squares for a starting board
        self.occupied_squares = {
            "white":[],
            "black":[]
        }
        for i in range(0,8):
            self.occupied_squares["white"].append(ascii_lowercase[i] + "1")
            self.occupied_squares["white"].append(ascii_lowercase[i] + "2")
            self.occupied_squares["black"].append(ascii_lowercase[i] + "7")
            self.occupied_squares["black"].append(ascii_lowercase[i] + "8")

    def run():
        if self.is_human_turn:
            while True:
                raw_input("It's your turn! Press Enter when you're done making your move.")
                (new_occupied_white_squares, new_occupied_black_squares) = CV_Interface.detect_board_state() #detect_board_state() returns a list of occupied squares detected by the camera

                new_occupied_squares = {
                    "white":new_occupied_white_squares,
                    "black":new_occupied_black_squares
                }

                end_squares = []
                start_squares = []
                
                #check if there are new occupied player squares.
                for square in new_occupied_squares[self.human_side]:
                    if square not in self.occupied_squares[self.human_side]:
                        end_squares.append(square)

                if len(end_squares) != 1:
                    print("Make sure to move exactly one piece. Try again!")
                    continue

                #check if there are new unoccupied player squares.
                for square in self.occupied_squares[self.human_side]:
                    if square not in new_occupied_squares[self.human_side]:
                        start_squares.append(square)

                if len(start_squares) != 1:
                    print("Make sure to move exactly one piece. Try again!")
                    continue

                #the move is the newly unoccupied square, followed by the newly occupied square.
                uci = (start_squares[0] + end_squares[0])
                print("Looks like you moved %s." %uci)

                move_is_legal = self.stockfish.make_human_move(uci) # TODO: implement promotions
                if move_is_legal:
                    if self.stockfish.is_game_over():
                        print("The game is over!")
                        return
                    self.is_human_turn = False
                    # TODO - how to move pieces?
                    break
                else:
                    print("That move was illegal. Try again.")
                    continue

        if not self.is_human_turn:
            #computer makes a move now.
            print("My move! Thinking...")
            stockfish_move = self.stockfish.make_stockfish_move()
            print("My move is " + stockfish_move)
            # TODO - how to move pieces?
            if self.stockfish.is_game_over():
                print("The game is over!")
                return

                

                
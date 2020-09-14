from StockfishInterface import StockfishInterface
from SerialInterface import SerialInterface
from string import ascii_lowercase
import cv_interface

class ChessBotCore():
    def __init__(self, human_is_white):
        self.is_human_turn = True

        #generate a set of occupied squares for a starting board
        self.white_occupied_squares = {}
        self.black_occupied_squares = {}
        for i in range(0,8):
            self.white_occupied_squares.append(ascii_lowercase[i] + "1")
            self.white_occupied_squares.append(ascii_lowercase[i] + "2")
            self.black_occupied_squares.append(ascii_lowercase[i] + "7")
            self.black_occupied_squares.append(ascii_lowercase[i] + "8")

        if human_is_white:
            self.is_white = False
        else:
            self.is_white = True

    def 

    def run():
        if self.is_human_turn:
            while True:
                raw_input("It's your turn! Press Enter when you're done making your move.")
                new_occupied_squares = set(cv_interface.detect_board_state()) #detect_board_state() returns a list of occupied squares detected by the camera
                change = 

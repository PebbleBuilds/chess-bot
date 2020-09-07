import chess #python-chess v0.23.8

class StockfishInterface():
    def __init__(self):
        self.engine = chess.uci.popen_engine("./stockfish_20090216_x64_bmi2")
        self.engine.uci()

        self.board = chess.Board()
    
    def new_game(self):
        self.engine.ucinewgame()
        self.board = chess.Board()

    def make_player_move(self, from_square, to_square, promotion=None):
        move = chess.Move(from_square, to_square, promotion)
        if move in self.board.legal_moves():
            self.board.push(move)

    def make_stockfish_move(self):
        if self.engine.isready():
            self.engine.position(self.board)
            (best_move, pondered_move) = self.engine.go(movetime=5000)
        if best_move is not None:
            self.board.push(best_move)
            return best_move.uci()
        return "0000"
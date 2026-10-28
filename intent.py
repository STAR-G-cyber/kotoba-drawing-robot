"""Japanese command classifier and differential-drive kinematic simulator.
Python standard library only. This is an educational prototype, not real robot control.
"""
import math
import re
import unicodedata
from collections import Counter
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parent
LABELS = {"face": "ロボットの顔（一部なぞり描き）", "diagonal_right": "右45度＋前進0.5 m", "diagonal_left": "左45度＋前進0.5 m", "square": "正方形走行（1辺0.5 m）", "forward": "前進 0.5 m", "backward": "後退 0.5 m", "left": "左旋回 90°", "right": "右旋回 90°", "stop": "停止", "spin": "左回り360度回転"}

def normalize(text):
    return re.sub(r"[\s、。!！?？]", "", unicodedata.normalize("NFKC", text).lower())

def features(text):
    text = normalize(text)
    return Counter(text[i:i+n] for n in (1, 2) for i in range(len(text)-n+1))

class Classifier:
    """Multinomial Naive Bayes, additive smoothing, character 1/2-grams."""
    def __init__(self):
        data = json.loads((ROOT / "training.json").read_text(encoding="utf-8-sig"))
        self.alpha = 0.5  # Additive smoothing; retain the 0.85 rejection threshold.
        self.counts = {label: Counter() for label in data}
        self.vocab = set()
        self.priors = {label: len(rows) for label, rows in data.items()}
        for label, rows in data.items():
            for row in rows:
                self.counts[label].update(features(row))
            self.vocab.update(self.counts[label])
        self.totals = {k: sum(v.values()) for k, v in self.counts.items()}

    def predict(self, text):
        normalized = normalize(text)
        # These are scope guards, not the learned intent classifier.
        if not normalized or len(normalized) > 80:
            return None, 0.0, "短い指示を1つ入力してください"
        guard_text = normalized.replace("360度", "")
        if re.search(r"ない|なく|ずに|禁止|だめ|ダメ|なよ|そして|それから|その後|から|[0-9]|cm|メートル|度|けど|ても|たら", guard_text):
            return None, 0.0, "否定・条件・連続指示、360度以外の数値指定は未対応です"
        if "右" in normalized and "左" in normalized:
            return None, 0.0, "右と左は1つずつ指示してください"
        backward = bool(re.search(r"後ろ|後退|バック|後方", normalized))
        if backward and re.search(r"前|直進|右|左|回転|一周|止ま|停止", normalized):
            return None, 0.0, "前進・後退・旋回・停止は1つずつ指示してください"
        if any(word in normalized for word in ("360度", "一周", "一回転")) and any(word in normalized for word in ("右", "前進", "進ん", "曲が", "停止", "止ま")):
            return None, 0.0, "360度回転は左回り1周のみです。指示を分けてください"
        tokens = features(text)
        known = {t: n for t, n in tokens.items() if t in self.vocab}
        coverage = sum(known.values()) / max(1, sum(tokens.values()))
        if coverage < 0.65:
            return None, 0.0, "知らない表現が多いため実行しません"
        scores = {}
        for label, counts in self.counts.items():
            # Diagonal movement must be explicitly requested.
            if label.startswith("diagonal_") and not re.search(r"斜め|ななめ", normalized):
                continue
            scores[label] = math.log(self.priors[label] / sum(self.priors.values()))
            scores[label] += sum(n * math.log((counts[t]+self.alpha)/(self.totals[label]+self.alpha*len(self.vocab))) for t, n in known.items())
        top = max(scores.values())
        probs = {k: math.exp(v-top) for k, v in scores.items()}
        total = sum(probs.values())
        label = max(probs, key=probs.get)
        score = probs[label] / total
        if label == "reject" or score < 0.85:
            return None, score, "判断できません。短い1動作に言い換えてください"
        return label, score, ""


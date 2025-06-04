from rouge_score import rouge_scorer
from nltk.translate.bleu_score import sentence_bleu

class Evaluator:
    def __init__(self):
        # Specify which ROUGE metrics you want
        self.rouge = rouge_scorer.RougeScorer(['rouge1', 'rouge2', 'rougeL'], use_stemmer=True)

    def evaluate_bleu(self, reference, candidate):
        reference_str = ' '.join([' '.join(step) for step in reference])
        candidate_str = ' '.join([' '.join(step) for step in candidate])
        reference_tokens = reference_str.split()
        candidate_tokens = candidate_str.split()
        score = sentence_bleu([reference_tokens], candidate_tokens)
        return score

    def evaluate_rouge(self, reference, candidate):
        reference_str = ' '.join([' '.join(step) for step in reference])
        candidate_str = ' '.join([' '.join(step) for step in candidate])
        scores = self.rouge.score(reference_str, candidate_str)
        return scores  # returns a dict with rouge1, rouge2, rougeL

if __name__ == "__main__":
    # Example usage
    evaluator = Evaluator()
    reference_plan = [
        ['lift_sequence', 'cube1'],
        ['move', 'cube2'],
        ['gripper_open']
    ]
    candidate_plan = [
        ['lift_sequence', 'cube1'],
        ['move', 'cube2'],
        ['gripper_close']
    ]
    bleu_score = evaluator.evaluate_bleu(reference_plan, candidate_plan)
    rouge_scores = evaluator.evaluate_rouge(reference_plan, candidate_plan)
    print(f"BLEU Score: {bleu_score}")
    print(f"ROUGE Scores: {rouge_scores}")
#     # Output:
#     # BLEU Score: 0.5773502691896257
#     # ROUGE Scores: {'rouge-1': {'f': 0.6666666666666666, 'p': 0.5, 'r': 1.0}, 'rouge-2': {'f': 0.3333333333333333, 'p': 0.25, 'r': 0.5}, 'rouge-l': {'f': 0.6666666666666666, 'p': 0.5, 'r': 1.0}}
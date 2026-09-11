#!/bin/bash
#SBATCH --job-name=ugv-100x100
#SBATCH --output=logs/%x_%j.out
#SBATCH --error=logs/%x_%j.err
#SBATCH --account=AGRIROBOT-KV6ERNX06SO-DEFAULT-CPU
#SBATCH --partition=cpu
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=16G
#SBATCH --time=4:00:00

export PYTHONUNBUFFERED=1

cd ~/UGV-Coordination
mkdir -p logs

source .venv/bin/activate

cd experiments/main_comparison

echo "== Sélection des instances =="
python3 select_instances.py \
  --config ../../config/experiments/main_comparison_validation_100x100.yaml \
  --tag validation100

echo "== Lancement de la comparaison =="
python3 run_comparison.py \
  --config ../../config/experiments/main_comparison_validation_100x100.yaml \
  --tag validation100

echo "✅ Job terminé"
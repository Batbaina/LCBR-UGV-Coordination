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
#SBATCH --time=24:00:00

set -e

export PYTHONUNBUFFERED=1

echo "=========================================="
echo "UGV CL-CBS vs LCBR — 100x100"
echo "Job ID: ${SLURM_JOB_ID}"
echo "Host: $(hostname)"
echo "Start: $(date)"
echo "=========================================="

cd "$HOME/UGV-Coordination"

mkdir -p logs

source .venv/bin/activate

echo
echo "Git commit:"
git rev-parse HEAD

echo
echo "== Vérification du binaire =="

if [ ! -x build/ugv_coordination ]; then
    echo "Binary absent — compilation..."

    mkdir -p build
    cd build
    cmake ..
    make -j"${SLURM_CPUS_PER_TASK}"
    cd ..
fi

./build/ugv_coordination --help >/dev/null

cd experiments/main_comparison

echo
echo "== Sélection déterministe des 550 instances =="

python3 select_instances.py \
    --config ../../config/experiments/main_comparison_final.yaml \
    --tag final50

echo
echo "== Extraction des instances 100x100 =="

grep "map100by100/" \
    selected_instances_final50.txt \
    > selected_instances_campaign100.txt

N=$(wc -l < selected_instances_campaign100.txt)

echo "Instances 100x100 sélectionnées: ${N}"

if [ "$N" -ne 200 ]; then
    echo "ERROR: 200 instances attendues, ${N} trouvées."
    exit 1
fi

echo
echo "== Lancement de campaign100 =="

python3 run_comparison.py \
    --config ../../config/experiments/main_comparison_final.yaml \
    --tag campaign100

echo
echo "=========================================="
echo "Campaign 100x100 terminée"
echo "End: $(date)"
echo "=========================================="
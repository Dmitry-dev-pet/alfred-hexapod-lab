"""Tablet/Termux: send the Google ADC one-time code to the Mac CI runner securely.

The plaintext authorization code stays only in Termux memory and is encrypted
with the Mac runner's freshly generated RSA-OAEP public key before GitHub sees it.
Never post the Google code directly into an Issue, chat, shell command or log.
"""

from __future__ import annotations

import base64
import getpass
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = "Dmytry-lab/runner-smoke"
ISSUE = 83
MARKER = "COLAB_ADC_CHALLENGE_V1"
RESPONSE_MARKER = "COLAB_ADC_RESPONSE_V1"
PATTERN = re.compile(
    r"^COLAB_ADC_CHALLENGE_V1\\nnonce=([a-f0-9]{24})\\n"
    r"public_key_pem_base64=([A-Za-z0-9+/=]+)\\n",
    re.MULTILINE,
)


def run_command(args: list[str], **kwargs: object) -> subprocess.CompletedProcess:
    return subprocess.run(args, check=True, **kwargs)


def main() -> int:
    try:
        who = run_command(["gh", "api", "user", "--jq", ".login"], capture_output=True, text=True)
        login = who.stdout.strip()
        if login != "Dmitry-dev-pet":
            print("Use gh auth switch/login for Dmitry-dev-pet first; current account:", login)
            return 2

        result = run_command(
            ["gh", "api", f"repos/{REPO}/issues/{ISSUE}/comments?per_page=100"],
            capture_output=True,
            text=True,
        )
        comments = json.loads(result.stdout)
    except (subprocess.CalledProcessError, OSError, ValueError) as exc:
        print("Cannot access private pairing Issue through gh. Error type:", type(exc).__name__)
        return 2

    matches = []
    for comment in comments:
        if (comment.get("user") or {}).get("login") != "github-actions[bot]":
            continue
        body = comment.get("body") or ""
        m = PATTERN.search(body)
        if m:
            matches.append((comment.get("created_at") or "", m.group(1), m.group(2)))
    if not matches:
        print("No active Mac handshake found in the private Issue; check GitHub Actions.")
        return 2
    matches.sort()
    _, nonce, pubkey_b64 = matches[-1]

    try:
        pem = base64.b64decode(pubkey_b64, validate=True)
        if b"BEGIN PUBLIC KEY" not in pem:
            print("Invalid MAC public key")
            return 2
    except ValueError:
        print("Invalid MAC public-key encoding")
        return 2

    print("Mac one-time pairing challenge found. Nonce:", nonce)
    print("Complete Google consent in Chrome using the URL in private Issue #83.")
    code = getpass.getpass("Paste Google's authorization code here (hidden): ").strip()
    if not code or len(code) > 300 or "\n" in code or "\r" in code:
        print("Authorization code missing or invalid length.")
        return 2

    try:
        with tempfile.TemporaryDirectory(prefix="colab-auth-public-key-") as tmp:
            keypath = Path(tmp) / "key.pem"
            keypath.write_bytes(pem)
            encrypted = run_command(
                [
                    "openssl", "pkeyutl", "-encrypt", "-pubin", "-inkey", str(keypath),
                    "-pkeyopt", "rsa_padding_mode:oaep",
                    "-pkeyopt", "rsa_oaep_md:sha256",
                ],
                input=code.encode("utf-8"),
                capture_output=True,
            ).stdout
        del code
        cipher = base64.b64encode(encrypted).decode("ascii")
        # Only a one-time RSA-OAEP ciphertext enters the GitHub comment body.
        post = f"{RESPONSE_MARKER}\\nnonce={nonce}\\nciphertext_base64={cipher}\\n"
        run_command(
            ["gh", "api", "-X", "POST", f"repos/{REPO}/issues/{ISSUE}/comments",
             "-f", f"body={post}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print("Encrypted one-time code sent. Only Mac's temporary private key can decrypt it.")
        return 0
    except (subprocess.CalledProcessError, OSError) as exc:
        print("Failed to send encrypted code. Error type:", type(exc).__name__)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())

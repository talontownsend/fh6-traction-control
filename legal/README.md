# Legal documents

Two different documents that are often confused, covering two different
things:

| File | Governs | Applies to |
|---|---|---|
| [`../LICENSE`](../LICENSE) | the source code in this repository | anyone reading or reusing the code |
| [`EULA.txt`](EULA.txt) | use of the built application | someone who installs the packaged app |
| [`THIRD-PARTY-NOTICES.txt`](THIRD-PARTY-NOTICES.txt) | components by other authors | shipped into the install directory |

The repository is licensed **PolyForm Noncommercial 1.0.0**: read it, modify
it, learn from it, use it for anything noncommercial. Commercial use is not
licensed.

The EULA is what the installer presents. It grants a personal, non
transferable licence to use the built application and forbids redistribution
and resale. It is deliberately not the same as the source licence, because a
source licence that permits redistribution would be the wrong thing to hand
someone as their terms of use.

## Standing caveats

**Not reviewed by a lawyer.** The EULA follows standard patterns and is much
better than nothing, but it has not had professional review.

**Consumer law usually wins.** Warranty disclaimers and liability caps are
frequently unenforceable against consumers, particularly in the EU and UK,
regardless of what any agreement says. Section 10 names Idaho as the governing
jurisdiction, but a consumer can generally still rely on their own local
protections.

**Third party notices are an obligation, not a courtesy.** `ViGEmClient.dll`
ships in the build under the BSD 3-Clause licence, whose second condition
requires reproducing its copyright notice and disclaimer in binary
redistributions. `THIRD-PARTY-NOTICES.txt` is installed alongside the
application to satisfy that. Do not drop it from the installer.

**Selling through a platform that acts as seller of record** (for example
Patreon, which collects and remits VAT and sales tax on pledges) offloads a
meaningful part of the tax and consumer exposure that would otherwise land on
an individual seller. A generous refund policy resolves most disputes before
they become anything else.

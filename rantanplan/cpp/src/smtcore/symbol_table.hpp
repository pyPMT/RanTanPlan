#pragma once

#include "smt_types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace rantanplan {

/**
 * @brief The interned store for named symbols -- variables and uninterpreted
 *        function symbols alike (in cvc5 a function symbol is just a `Term`
 *        whose sort is a function sort; the discriminator is
 *        `t.getSort().isFunction()`).
 *
 * Interning is the identity mechanism, not an optimization: mkConst mints a
 * fresh variable on every call ("even if the same arguments were provided on a
 * previous call", cvc5.h:4388), so a SymbolTable being the only place named
 * symbols are created is what makes two occurrences of one name the same Term.
 *
 * Each instance is bound by use to one TermManager (its entries are only
 * meaningful there) and its owner defines the identity scope: the
 * SmtVariableFactory holds one for the encoding's lifetime, while analyses
 * (the symmetry checker) own clearable per-query tables.
 */
class SymbolTable {
public:
    /// Get-or-create a plain (non-function) variable. If `name` is cached as
    /// a function symbol, a fresh variable is minted on every call -- a
    /// variable/fluent name collision is not handled, only not known to occur.
    Term intern_var(cvc5::TermManager& tm, const std::string& name,
                    const cvc5::Sort& sort) {
        auto it = table_.find(name);
        if (it != table_.end() && !it->second.getSort().isFunction()) {
            return it->second;
        }
        Term var = tm.mkConst(sort, name);
        table_.emplace(name, var);  // no-op on the collision case
        return var;
    }

    /// Get-or-create a function symbol. A 0-ary "function" is a plain
    /// constant (mkFunctionSort rejects an empty domain), so a cached
    /// non-function entry is returned as-is -- the legitimate 0-ary case and
    /// a symbol/fluent name collision are indistinguishable here, and the
    /// 0-ary case is reachable in normal use.
    Term intern_function(cvc5::TermManager& tm, const std::string& name,
                         const std::vector<cvc5::Sort>& domain,
                         const cvc5::Sort& range) {
        auto it = table_.find(name);
        if (it != table_.end()) return it->second;
        Term sym = domain.empty()
            ? tm.mkConst(range, name)
            : tm.mkConst(tm.mkFunctionSort(domain, range), name);
        table_.emplace(name, sym);
        return sym;
    }

    void clear() { table_.clear(); }

private:
    std::unordered_map<std::string, Term> table_;
};

}  // namespace rantanplan

use crate::GcodeError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Word {
    pub letter: char,
    pub value: f64,
}

/// One line of G-code.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Block {
    /// 1-based source line number.
    pub line: usize,
    pub words: Vec<Word>,
    pub comment: Option<String>,
    pub block_delete: bool,
}

impl Block {
    pub fn get(&self, letter: char) -> Option<f64> {
        self.words.iter().find(|w| w.letter == letter).map(|w| w.value)
    }

    pub fn has(&self, letter: char) -> bool {
        self.words.iter().any(|w| w.letter == letter)
    }

    /// All G (or M) codes in the block, as values ×10 rounded so G91.1 = 911.
    pub fn codes(&self, letter: char) -> impl Iterator<Item = i32> + '_ {
        self.words
            .iter()
            .filter(move |w| w.letter == letter)
            .map(|w| (w.value * 10.0).round() as i32)
    }
}

pub fn parse(src: &str) -> Result<Vec<Block>, GcodeError> {
    src.lines().enumerate().map(|(i, l)| parse_line(l, i + 1)).collect()
}

pub fn parse_line(text: &str, line: usize) -> Result<Block, GcodeError> {
    let err = |msg: String| GcodeError::Parse { line, msg };
    let mut b = Block { line, ..Default::default() };
    let chars: Vec<char> = text.chars().collect();
    let mut i = 0;
    let mut comments = Vec::new();

    while i < chars.len() && chars[i].is_whitespace() {
        i += 1;
    }
    if i < chars.len() && chars[i] == '/' {
        b.block_delete = true;
        i += 1;
    }

    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() || c == '%' {
            i += 1;
        } else if c == '(' {
            let end = chars[i..].iter().position(|&c| c == ')').map(|p| p + i).unwrap_or(chars.len());
            comments.push(chars[i + 1..end].iter().collect::<String>());
            i = end + 1;
        } else if c == ';' {
            comments.push(chars[i + 1..].iter().collect::<String>());
            break;
        } else if c.is_ascii_alphabetic() {
            let letter = c.to_ascii_uppercase();
            i += 1;
            while i < chars.len() && chars[i] == ' ' {
                i += 1;
            }
            let start = i;
            if i < chars.len() && (chars[i] == '-' || chars[i] == '+') {
                i += 1;
            }
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            let num: String = chars[start..i].iter().collect();
            let value: f64 = num.parse().map_err(|_| err(format!("bad number after '{letter}': '{num}'")))?;
            b.words.push(Word { letter, value });
        } else {
            return Err(err(format!("unexpected character '{c}'")));
        }
    }
    if !comments.is_empty() {
        b.comment = Some(comments.join(" ").trim().to_string());
    }
    // Line numbers carry no meaning for execution.
    b.words.retain(|w| w.letter != 'N');
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_words_and_comments() {
        let b = parse_line("N10 G1 X-1.5 y2 F 300 (cut) ; tail", 1).unwrap();
        assert_eq!(b.get('X'), Some(-1.5));
        assert_eq!(b.get('Y'), Some(2.0));
        assert_eq!(b.get('F'), Some(300.0));
        assert_eq!(b.comment.as_deref(), Some("cut  tail"));
        assert!(!b.has('N'));
    }

    #[test]
    fn decimal_g_codes() {
        let b = parse_line("G91.1 G17", 1).unwrap();
        assert_eq!(b.codes('G').collect::<Vec<_>>(), vec![911, 170]);
    }
}

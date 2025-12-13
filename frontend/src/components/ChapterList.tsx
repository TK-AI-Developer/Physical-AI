import React, { useEffect, useState } from 'react';
import axios from 'axios';

interface Chapter {
  title: string;
  slug: string;
  summary: string;
}

function ChapterList(): JSX.Element {
  const [chapters, setChapters] = useState<Chapter[]>([]);

  useEffect(() => {
    axios.get<Chapter[]>('http://localhost:8000/chapters')
      .then(response => {
        setChapters(response.data);
      })
      .catch(error => {
        console.error('Error fetching chapters:', error);
      });
  }, []);

  return (
    <div>
      <h1>Chapters</h1>
      <ul>
        {chapters.map(chapter => (
          <li key={chapter.slug}>
            <a href={`/docs/${chapter.slug}`}>{chapter.title}</a>
            <p>{chapter.summary}</p>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default ChapterList;
